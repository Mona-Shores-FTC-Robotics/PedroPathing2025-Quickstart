package org.firstinspires.ftc.teamcode.flywheel;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Flywheel tuner OpMode with:
 *  - 200 Hz custom PIDF control (separate thread)
 *  - Panels live tuning (edit FlywheelConfig while running)
 *  - AdvantageScope-Lite live plots via FTC Dashboard
 *
 * Gamepad shortcuts (convenience):
 *   RB / LB : increase / decrease FlywheelConfig.targetRpm
 *   A       : set target to 0 rpm
 *   B       : set target to 3000 rpm (change if you want)
 */
@TeleOp(name = "FlywheelTuner-200Hz (Readable)", group = "Test")
public class FlywheelTunerOpMode extends LinearOpMode {

    // --- Hardware + controller ---
    private FlywheelHardware flywheel;
    private FlywheelPIDFController pidf;

    // --- UI / Telemetry ---
    private TelemetryManager panels;        // your PanelsTelemetry
    private FlywheelLogSink livePlotSink;   // DashboardLogSink (AdvantageScope-Lite)
    private FlywheelLogSink persistentSink; // WpiLogSink (stubbed for now)

    // --- Shared values between threads (volatile = safe simple sharing) ---
    private volatile double lastMeasuredTicksPerSecond = 0.0;
    private volatile double lastSetpointTicksPerSecond = 0.0;
    private volatile double lastMotorPower0to1 = 0.0;
    private volatile boolean runFastLoop = false;

    @Override
    public void runOpMode() {
        // 1) Initialize telemetry
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // 2) Initialize hardware
        // CHANGE these to match your robot config:
        // Example: 28 CPR × 20:1 gearbox × 4x quadrature = 2240 ticks/rev (no external ratio).
        final int ticksPerRevolution = 28 * 20 * 4;  // <-- set correctly for your hardware!
        flywheel = new FlywheelHardware(hardwareMap, "flywheel", ticksPerRevolution);

        // 3) Initialize controller
        pidf = new FlywheelPIDFController();
        flywheel.clearBulkCache();
        double initialVelocityTps = flywheel.getVelocityTicksPerSecond();
        pidf.reset(initialVelocityTps);
        lastMeasuredTicksPerSecond = initialVelocityTps;

        // 4) Set up logging sinks
        livePlotSink = new DashboardLogSink();      // live graphs
        persistentSink = new WpiLogSink(false);     // set true after wiring KoalaLog/PsiKit

        telemetry.addLine("Flywheel tuner ready.");
        telemetry.addLine("RB/LB = +/- target RPM | A=0 | B=3000");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 5) Start fast control loop (200 Hz) on a separate thread
        runFastLoop = true;
        Thread controlThread = new Thread(this::fastControlLoop200Hz, "FlywheelControl200Hz");
        controlThread.setPriority(Thread.NORM_PRIORITY + 1);
        controlThread.start();

        // 6) UI loop: run ~50 Hz for Panels + simple RC telemetry
        while (opModeIsActive() && !isStopRequested()) {

            // Convenience target tweaks from the gamepad (Panels values still live-update)
            if (gamepad1.right_bumper) FlywheelConfig.targetRpm += FlywheelConfig.targetStepRpm;
            if (gamepad1.left_bumper)  FlywheelConfig.targetRpm -= FlywheelConfig.targetStepRpm;
            if (gamepad1.a)            FlywheelConfig.targetRpm = 0;
            if (gamepad1.b)            FlywheelConfig.targetRpm = 3000;

            // Panels quick status
            panels.debug("Target RPM", Math.round(FlywheelConfig.targetRpm));
            panels.debug("Measured RPM", Math.round(flywheel.ticksPerSecondToRpm(lastMeasuredTicksPerSecond)));
            panels.debug("Motor Power", String.format("%.3f", lastMotorPower0to1));

            // Optional: let persistent sink flush if needed
            persistentSink.periodic();

            // Simple RC telemetry (so you can see something without opening the web UI)
            telemetry.addData("TargetRPM", Math.round(FlywheelConfig.targetRpm));
            telemetry.addData("RPM", Math.round(flywheel.ticksPerSecondToRpm(lastMeasuredTicksPerSecond)));
            telemetry.addData("Power", String.format("%.3f", lastMotorPower0to1));
            telemetry.update();

            sleep(20); // ~50 Hz
        }

        // 7) Clean shutdown
        runFastLoop = false;
        try { controlThread.join(300); } catch (InterruptedException ignored) {}
        flywheel.setMotorPower(0.0);
        persistentSink.close();
    }

    /**
     * High-rate loop (200 Hz):
     *  read sensors → compute setpoint → compute power → command motor → publish shared values → log.
     *  Keep this loop light; never call SDK telemetry here.
     */
    private void fastControlLoop200Hz() {
        final long periodNanos = 5_000_000L; // 5 ms → 200 Hz
        long nextTimeNanos = System.nanoTime();

        while (runFastLoop) {
            long now = System.nanoTime();
            if (now < nextTimeNanos) {
                long sleepNs = nextTimeNanos - now;
                if (sleepNs > 2_000_000) {
                    try { Thread.sleep(sleepNs / 1_000_000L, (int) (sleepNs % 1_000_000L)); }
                    catch (InterruptedException ignored) {}
                } else {
                    Thread.yield();
                }
                continue;
            }
            nextTimeNanos += periodNanos;

            // --- 1) Read current velocity (ticks/second) ---
            flywheel.clearBulkCache();
            double measuredTps = flywheel.getVelocityTicksPerSecond();

            // --- 2) Build setpoint (ticks/second) from Panels target RPM, with ramping ---
            double dtSeconds = periodNanos / 1e9;
            double requestedTps = flywheel.rpmToTicksPerSecond(FlywheelConfig.targetRpm);
            double setpointTps = pidf.rampSetpointTicksPerSecond(requestedTps, dtSeconds);

            // --- 3) PIDF → motor power ---
            double power0to1 = pidf.calculatePower(measuredTps, setpointTps, dtSeconds);
            flywheel.setMotorPower(power0to1);

            // --- 4) Share values for UI loop (volatiles = simple + safe) ---
            lastMeasuredTicksPerSecond = measuredTps;
            lastSetpointTicksPerSecond = setpointTps;
            lastMotorPower0to1 = power0to1;

            // --- 5) Log (live plot + optional persistent) ---
            long nowMs = System.currentTimeMillis();
            double rpm = flywheel.ticksPerSecondToRpm(measuredTps);
            double targetRpm = flywheel.ticksPerSecondToRpm(setpointTps);
            double errorRpm = targetRpm - rpm;

            livePlotSink.log(nowMs, rpm, targetRpm, power0to1, errorRpm, measuredTps, setpointTps);
            persistentSink.log(nowMs, rpm, targetRpm, power0to1, errorRpm, measuredTps, setpointTps);
        }
    }
}

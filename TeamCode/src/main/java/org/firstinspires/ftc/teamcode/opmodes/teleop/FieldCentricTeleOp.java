package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;

/**
 * Clean teleop OpMode demonstrating field‑centric drive with auto‑heading and
 * a bang‑bang flywheel.  The right bumper enables slow mode for fine
 * manoeuvres.  Press A to spin up the flywheel to a default speed and B to
 * stop it.
 */
@TeleOp(name = "FieldCentricTeleOp (Clean)", group = "Refactor")
public class FieldCentricTeleOp extends OpMode {

    private DriveSubsystem drive;
    private FlywheelSubsystem fly;
    private TelemetryPublisher pub;
    private org.firstinspires.ftc.teamcode.util.PsiKitAdapter logger;

    // Flywheel toggling: track whether the flywheel is currently commanded on
    private boolean flyOn = false;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        fly   = new FlywheelSubsystem(hardwareMap);
        // Start logging: create a PsiKit adapter and start a session.  The
        // logger writes a CSV file to the SD card that can be opened in
        // AdvantageScope.  Passing the logger to the publisher will cause all
        // telemetry to be recorded to the file as well as sent to the dashboard.
        logger = new org.firstinspires.ftc.teamcode.util.PsiKitAdapter();
        logger.startSession();
        pub   = new TelemetryPublisher(logger);
        // Set an initial pose for telemetry.  Modify as appropriate for your field.
        drive.setPose(0.0, 0.0, 0.0);
    }

    @Override
    public void loop() {
        // Read joystick inputs (note: forward stick is negative Y on the gamepad)
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        // Slow mode enabled while right bumper is held
        boolean slowHold = gamepad1.right_bumper;
        drive.setSlowMode(slowHold);

        // Drive subsystem handles auto‑heading when not in slow mode
        drive.drive(lx, ly, rx);

        // Flywheel control: A starts, B stops
        if (gamepad1.a && !flyOn) {
            fly.setTargetRpm(Constants.FLY_TEST_RPM);
            flyOn = true;
        } else if (gamepad1.b && flyOn) {
            fly.stop();
            flyOn = false;
        }

        fly.controlLoopOnce();

        // Publish telemetry to FTC Dashboard
        pub.publishDrive(drive, lx, ly, rx, slowHold);
        // Flywheel telemetry: include the last power applied by the control loop
        double rpm = fly.getRpm();
        double err = fly.getTargetRpm() - rpm;
        pub.publishFlywheel(fly.getTargetRpm(), rpm, fly.getLastPower(), err);

        // Also show basic telemetry on the driver station phone
        telemetry.addData("slowMode", slowHold);
        telemetry.addData("pose", "(%.1f, %.1f) h=%.1f°",
                drive.getPose().x, drive.getPose().y,
                Math.toDegrees(drive.getPose().headingRad));
        telemetry.addData("flywheel", "%.0f / %.0f rpm", rpm, fly.getTargetRpm());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (fly != null) {
            fly.stop();
        }
        if (logger != null) {
            logger.stopSession();
        }
    }
}

package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Talks to the single flywheel motor and computes velocity in ticks/second
 * using REV bulk data for high-rate updates.
 */
public class FlywheelHardware {

    private final DcMotorEx motor;
    private final List<LynxModule> hubs;
    private final int ticksPerRevolution;

    // State used to compute velocity
    private int previousEncoderTicks;
    private long previousTimestampNanos;
    private double filteredVelocityTicksPerSecond = 0.0;

    public FlywheelHardware(HardwareMap hardwareMap,
                            String motorConfigName,
                            int ticksPerRevolution) {
        this.ticksPerRevolution = ticksPerRevolution;

        motor = hardwareMap.get(DcMotorEx.class, motorConfigName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we control power manually
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotor.Direction.FORWARD);        // flip if needed for your build

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        previousEncoderTicks = motor.getCurrentPosition();
        previousTimestampNanos = System.nanoTime();
    }

    /** MUST be called once per fast update before reading positions. */
    public void clearBulkCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    /**
     * @return measured speed in ticks/second (EMA filtered for readability).
     */
    public double getVelocityTicksPerSecond() {
        int currentTicks = motor.getCurrentPosition();
        long nowNanos = System.nanoTime();

        double dtSeconds = Math.max(1e-3, (nowNanos - previousTimestampNanos) / 1e9);
        double rawTicksPerSecond = (currentTicks - previousEncoderTicks) / dtSeconds;

        // Exponential moving average to reduce quantization noise
        double a = clamp01(FlywheelConfig.velocityEmaAlpha);
        filteredVelocityTicksPerSecond = a * rawTicksPerSecond + (1 - a) * filteredVelocityTicksPerSecond;

        previousEncoderTicks = currentTicks;
        previousTimestampNanos = nowNanos;

        return filteredVelocityTicksPerSecond;
    }

    public void setMotorPower(double power0to1) {
        motor.setPower(power0to1);
    }

    // --- convenience conversions ---
    public double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond / ticksPerRevolution) * 60.0;
    }

    public double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * ticksPerRevolution;
    }

    // --- helpers ---
    private static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }
}

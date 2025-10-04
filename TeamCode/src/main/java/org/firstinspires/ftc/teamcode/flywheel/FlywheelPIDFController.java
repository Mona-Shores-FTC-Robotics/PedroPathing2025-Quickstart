package org.firstinspires.ftc.teamcode.flywheel;

/**
 * PIDF (kS + kV + kA + PID) controller for flywheel speed.
 * Inputs/outputs are in clear units. No hardware here—just math.
 */
public class FlywheelPIDFController {

    private double previousVelocityTicksPerSecond = 0.0;
    private double previousSlewSetpointTicksPerSecond = 0.0;
    private double integralTerm = 0.0;

    /** Call when starting or after disabling control. */
    public void reset(double currentVelocityTicksPerSecond) {
        previousVelocityTicksPerSecond = currentVelocityTicksPerSecond;
        previousSlewSetpointTicksPerSecond = currentVelocityTicksPerSecond;
        integralTerm = 0.0;
    }

    /**
     * Softly ramps the target setpoint to avoid current spikes and overshoot.
     */
    public double rampSetpointTicksPerSecond(double requestedTicksPerSecond, double dtSeconds) {
        double maxDelta = FlywheelConfig.maxSetpointAccelTicksPerSecond2 * dtSeconds;
        double delta = clamp(
                requestedTicksPerSecond - previousSlewSetpointTicksPerSecond,
                -maxDelta, maxDelta
        );
        previousSlewSetpointTicksPerSecond += delta;
        return previousSlewSetpointTicksPerSecond;
    }

    /**
     * Compute motor power (0..1).
     *
     * @param measuredVelocityTps   current speed in ticks/second
     * @param setpointVelocityTps   desired speed in ticks/second (after ramping)
     * @param dtSeconds             loop period in seconds
     */
    public double calculatePower(double measuredVelocityTps,
                                 double setpointVelocityTps,
                                 double dtSeconds) {

        // ---- Feedforward ----
        double accelEstimateTps2 = (measuredVelocityTps - previousVelocityTicksPerSecond) / Math.max(dtSeconds, 1e-3);
        double ffPower = 0.0;
        if (Math.abs(setpointVelocityTps) > 50.0) { // ignore tiny targets to keep kS from kicking on noise
            ffPower = FlywheelConfig.kS
                    + FlywheelConfig.kV * setpointVelocityTps
                    + FlywheelConfig.kA * accelEstimateTps2;
        }

        // ---- PID terms ----
        double errorTps = setpointVelocityTps - measuredVelocityTps;

        // Integral with clamp to prevent windup
        integralTerm += errorTps * dtSeconds;
        integralTerm = clamp(integralTerm, -FlywheelConfig.integralClampAbs, FlywheelConfig.integralClampAbs);

        // Derivative on measurement (robust to step changes in setpoint)
        double derivativeOnMeasurementTps2 =
                (measuredVelocityTps - previousVelocityTicksPerSecond) / Math.max(dtSeconds, 1e-3);

        double pidPower =
                FlywheelConfig.kP * errorTps +
                FlywheelConfig.kI * integralTerm -
                FlywheelConfig.kD * derivativeOnMeasurementTps2;

        // ---- Output clamp ----
        double power = clamp(ffPower + pidPower, FlywheelConfig.powerMin, FlywheelConfig.powerMax);

        // ---- Remember last values for next iteration ----
        previousVelocityTicksPerSecond = measuredVelocityTps;
        return power;
    }

    // --- helpers ---
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing.PIDFController;

import java.util.function.DoubleSupplier;

/**
 * A PIDF (Proportional, Integral, Derivative, Feedforward) controller for motor velocity.
 * This controller is designed for motors that need to attain and maintain a certain velocity.
 * It uses PID to correct velocity errors and feedforward to anticipate the required power.
 */
public class VelocityPIDFController {

    // Gains
    private double kP, kI, kD, kS, kV;

    // State variables
    private double targetVelocity;
    private double lastError = 0;
    private double integralSum = 0;
    private double lastTimeNanos = 0;
    private double lastVelocity = 0;
    private double lastOutput = 0;

    // Clamping limits
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    private double minIntegral = -1.0;
    private double maxIntegral = 1.0;

    // Advanced features
    private DoubleSupplier batteryVoltageSupplier;
    private double nominalVoltage = 12.0;
    private double kaw = 0.0; // Anti-windup back-calculation gain

    private static final double VELOCITY_EPSILON = 1e-6;

    /**
     * Minimal constructor for a velocity PID controller.
     *
     * @param kP Proportional gain (on velocity error).
     * @param kI Integral gain (on velocity error).
     * @param kD Derivative gain (on change in velocity error).
     */
    public VelocityPIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0);
    }

    /**
     * Constructor for a velocity PIDF controller with feedforward.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kS Static friction feedforward gain.
     * @param kV Velocity feedforward gain (applied to target velocity).
     */
    public VelocityPIDFController(double kP, double kI, double kD, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
    }

    /**
     * Computes the controller output to achieve the target velocity.
     *
     * @param currentVelocity The current velocity of the motor.
     * @param targetVelocity The desired velocity.
     * @return The computed controller output, clamped within the output limits.
     */
    public double compute(double currentVelocity, double targetVelocity) {
        this.targetVelocity = targetVelocity;

        long currentTimeNanos = System.nanoTime();
        if (lastTimeNanos == 0) {
            lastTimeNanos = currentTimeNanos;
            lastVelocity = currentVelocity;
            return 0;
        }

        double dt = (currentTimeNanos - lastTimeNanos) * 1e-9;
        if (dt <= 0) {
            return lastOutput;
        }

        double error = targetVelocity - currentVelocity;

        // Proportional term
        double p = kP * error;

        // Integral term
        integralSum += kI * error * dt;

        // Derivative term (on measurement)
        double derivative = (currentVelocity - lastVelocity) / dt;
        double d = -kD * derivative;

        // Feedforward term
        double ff = kV * targetVelocity;
        if (Math.abs(targetVelocity) > VELOCITY_EPSILON) {
            ff += kS * Math.signum(targetVelocity);
        }

        // Apply voltage compensation to feedforward term if enabled
        if (batteryVoltageSupplier != null) {
            ff *= nominalVoltage / batteryVoltageSupplier.getAsDouble();
        }

        // Combine terms
        double unsatOutput = p + integralSum + d + ff;

        // Clamp output
        double output = clamp(unsatOutput, minOutput, maxOutput);

        // Anti-windup back-calculation
        if (kaw != 0) {
            integralSum += kaw * (output - unsatOutput) * dt;
        }

        // Clamp integral sum
        integralSum = clamp(integralSum, minIntegral, maxIntegral);

        // Update state for next iteration
        lastError = error;
        lastVelocity = currentVelocity;
        lastTimeNanos = currentTimeNanos;
        lastOutput = output;

        return output;
    }

    /**
     * Resets the controller's internal state.
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTimeNanos = 0;
        lastVelocity = 0;
        lastOutput = 0;
    }

    // Setters and Getters

    /**
     * Sets the PID gains.
     */
    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Sets the feedforward gains.
     */
    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }

    /**
     * Sets the output limits.
     */
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
     * Sets the integral clamp limits.
     */
    public void setIntegralLimits(double min, double max) {
        this.minIntegral = min;
        this.maxIntegral = max;
    }

    /**
     * Enables voltage compensation for the feedforward term.
     */
    public void enableVoltageCompensation(double nominalVoltage, DoubleSupplier batteryVoltageSupplier) {
        this.nominalVoltage = nominalVoltage;
        this.batteryVoltageSupplier = batteryVoltageSupplier;
    }

    /**
     * Disables voltage compensation.
     */
    public void disableVoltageCompensation() {
        this.batteryVoltageSupplier = null;
    }

    /**
     * Sets the anti-windup back-calculation gain.
     */
    public void setIntegratorBackCalculation(double kaw) {
        this.kaw = kaw;
    }

    /**
     * Sets the target velocity.
     */
    public void setTarget(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getTarget() { return targetVelocity; }
    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkS() { return kS; }
    public double getkV() { return kV; }

    // Telemetry helpers
    public double getLastDt() { return (System.nanoTime() - lastTimeNanos) * 1e-9; }
    public double getLastError() { return lastError; }
    public double getLastOutput() { return lastOutput; }
    public double getIntegralSum() { return integralSum; }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}

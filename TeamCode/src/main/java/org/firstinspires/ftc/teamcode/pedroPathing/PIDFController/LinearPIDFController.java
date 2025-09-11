package org.firstinspires.ftc.teamcode.pedroPathing.PIDFController;

import java.util.function.DoubleSupplier;

/**
 * A PIDF (Proportional, Integral, Derivative, Feedforward) controller for linear mechanisms.
 * This controller is designed for mechanisms like slides or linear actuators. It operates with
 * internal timing and provides features like derivative-on-measurement, anti-windup,
 * feedforward terms (kS, kV, kA), output clamping, and optional voltage compensation.
 *
 * The controller's units are agnostic and depend on the user's input units for position and velocity.
 */
public class LinearPIDFController {

    // Gains
    private double kP, kI, kD, kS, kV, kA;

    // State variables
    private double targetPosition;
    private double lastError = 0;
    private double integralSum = 0;
    private double lastPosition = 0;
    private long lastTimeNanos = 0;
    private double lastVelocity = 0;
    private double lastOutput = 0;

    // Clamping limits
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    private double minIntegral = -1.0;
    private double maxIntegral = 1.0;

    // Advanced features
    private double derivativeFilterAlpha = 1.0; // Default to no filtering
    private DoubleSupplier batteryVoltageSupplier;
    private double nominalVoltage = 12.0;
    private double staticDeadband = 0.0;
    private double kaw = 0.0; // Anti-windup back-calculation gain

    private static final double VELOCITY_EPSILON = 1e-6;

    /**
     * Minimal constructor for a PID controller.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     */
    public LinearPIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0);
    }

    /**
     * Constructor for a PIDF controller with velocity and static friction feedforward.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kS Static friction feedforward gain.
     * @param kV Velocity feedforward gain.
     */
    public LinearPIDFController(double kP, double kI, double kD, double kS, double kV) {
        this(kP, kI, kD, kS, kV, 0);
    }

    /**
     * Full constructor for a PIDF controller with all feedforward terms.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kS Static friction feedforward gain.
     * @param kV Velocity feedforward gain.
     * @param kA Acceleration feedforward gain.
     */
    public LinearPIDFController(double kP, double kI, double kD, double kS, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Computes the controller output using the current and target positions.
     * Velocity is estimated internally.
     *
     * @param currentPosition The current position of the mechanism.
     * @param targetPosition  The target position of the mechanism.
     * @return The computed controller output, clamped within the output limits.
     */
    public double compute(double currentPosition, double targetPosition) {
        this.targetPosition = targetPosition;

        long currentTimeNanos = System.nanoTime();
        if (lastTimeNanos == 0) {
            lastTimeNanos = currentTimeNanos;
            lastPosition = currentPosition;
            return 0;
        }

        double dt = (currentTimeNanos - lastTimeNanos) * 1e-9;
        if (dt <= 0) {
             // To prevent division by zero and other issues on the first loop or with clock issues
            return lastOutput;
        }

        double velocity = (currentPosition - lastPosition) / dt;
        double filteredVelocity = derivativeFilterAlpha * velocity + (1 - derivativeFilterAlpha) * lastVelocity;

        return computeWithVelocity(currentPosition, targetPosition, filteredVelocity, dt);
    }

    /**
     * Computes the controller output using an explicit measured velocity.
     * This bypasses the internal velocity estimation.
     *
     * @param currentPosition The current position of the mechanism.
     * @param targetPosition  The target position of the mechanism.
     * @param measuredVelocity The externally measured velocity.
     * @return The computed controller output.
     */
    public double computeWithVelocity(double currentPosition, double targetPosition, double measuredVelocity) {
        this.targetPosition = targetPosition;

        long currentTimeNanos = System.nanoTime();
        if (lastTimeNanos == 0) {
            lastTimeNanos = currentTimeNanos;
            return 0;
        }
        double dt = (currentTimeNanos - lastTimeNanos) * 1e-9;

        return computeWithVelocity(currentPosition, targetPosition, measuredVelocity, dt);
    }

    /**
     * Internal computation logic shared by public compute methods.
     */
    private double computeWithVelocity(double currentPosition, double targetPosition, double velocity, double dt) {
        double error = targetPosition - currentPosition;

        // Proportional term
        double p = kP * error;

        // Integral term update
        integralSum += kI * error * dt;

        // Derivative term (on measurement)
        double d = -kD * velocity;

        // Feedforward term
        double ff = calculateFeedforward(velocity, 0); // No acceleration target for now

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
        lastPosition = currentPosition;
        lastTimeNanos = System.nanoTime();
        lastVelocity = velocity;
        lastOutput = output;

        return output;
    }

    /**
     * Calculates the feedforward term.
     *
     * @param velocity The current velocity.
     * @param acceleration The target acceleration (currently unused, for kA).
     * @return The calculated feedforward value.
     */
    protected double calculateFeedforward(double velocity, double acceleration) {
        double ff = kV * velocity + kA * acceleration;
        if (Math.abs(lastOutput) > staticDeadband) {
            if (Math.abs(velocity) > VELOCITY_EPSILON) {
                ff += kS * Math.signum(velocity);
            } else {
                ff += kS * Math.signum(lastError);
            }
        }
        return ff;
    }

    /**
     * Resets the controller's internal state (integrator, filters, time).
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastPosition = 0;
        lastTimeNanos = 0;
        lastVelocity = 0;
        lastOutput = 0;
    }

    // Setters and Getters

    /**
     * Sets the PID gains.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     */
    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Sets the feedforward gains.
     *
     * @param kS Static friction gain.
     * @param kV Velocity feedforward gain.
     */
    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }

    /**
     * Sets the feedforward gains including acceleration.
     *
     * @param kS Static friction gain.
     * @param kV Velocity feedforward gain.
     * @param kA Acceleration feedforward gain.
     */
    public void setFeedforward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Sets the output limits.
     *
     * @param min Minimum output, typically -1.
     * @param max Maximum output, typically 1.
     */
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
     * Sets the integral clamp limits.
     *
     * @param min Minimum integral sum.
     * @param max Maximum integral sum.
     */
    public void setIntegralLimits(double min, double max) {
        this.minIntegral = min;
        this.maxIntegral = max;
    }

    /**
     * Sets the alpha for the low-pass filter on the derivative term.
     * A value of 1.0 means no filtering.
     *
     * @param alpha The filter coefficient (0 < alpha <= 1).
     */
    public void setDerivativeFilterAlpha(double alpha) {
        this.derivativeFilterAlpha = clamp(alpha, 0, 1);
    }

    /**
     * Sets the derivative low-pass filter based on a cutoff frequency.
     *
     * @param cutoffHz The cutoff frequency in Hz.
     * @param loopTimeHint The estimated loop time in seconds.
     */
    public void setDerivativeCutoffHz(double cutoffHz, double loopTimeHint) {
        if (cutoffHz <= 0) {
            this.derivativeFilterAlpha = 1.0; // No filtering
            return;
        }
        double rc = 1.0 / (2 * Math.PI * cutoffHz);
        this.derivativeFilterAlpha = loopTimeHint / (rc + loopTimeHint);
    }

    /**
     * Enables voltage compensation for the feedforward term.
     *
     * @param nominalVoltage The nominal voltage of the system (e.g., 12.0V).
     * @param batteryVoltageSupplier A supplier for the current battery voltage.
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
     * Sets a deadband around zero output where the static friction term (kS) is not applied.
     *
     * @param deadband The command output deadband.
     */
    public void setStaticDeadband(double deadband) {
        this.staticDeadband = Math.abs(deadband);
    }

    /**
     * Sets the anti-windup back-calculation gain.
     * Setting this to 0 disables back-calculation.
     *
     * @param kaw The back-calculation gain.
     */
    public void setIntegratorBackCalculation(double kaw) {
        this.kaw = kaw;
    }

    /**
     * Sets the target position for the controller.
     * @param targetPosition The desired target position.
     */
    public void setTarget(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getTarget() { return targetPosition; }
    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkS() { return kS; }
    public double getkV() { return kV; }
    public double getkA() { return kA; }

    // Telemetry helpers
    public double getLastDt() { return (System.nanoTime() - lastTimeNanos) * 1e-9; }
    public double getLastError() { return lastError; }
    public double getLastVelocity() { return lastVelocity; }
    public double getLastOutput() { return lastOutput; }
    public double getIntegralSum() { return integralSum; }

    /**
     * Clamps a value between a minimum and maximum.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}

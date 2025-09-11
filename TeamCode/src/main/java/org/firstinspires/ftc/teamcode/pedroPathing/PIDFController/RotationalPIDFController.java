package org.firstinspires.ftc.teamcode.pedroPathing.PIDFController;

import java.util.function.DoubleSupplier;

/**
 * A PIDF (Proportional, Integral, Derivative, Feedforward) controller for rotational mechanisms.
 * This controller is designed for arms or turrets and includes a gravity feedforward term (kG).
 * It inherits the core logic from LinearPIDFController and adds rotational-specific features.
 */
public class RotationalPIDFController extends LinearPIDFController {

    private double kG;
    private double currentAngleRad;

    /**
     * Minimal constructor for a rotational PID controller.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     */
    public RotationalPIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0);
    }

    /**
     * Constructor for a rotational PIDF controller with kS and kV feedforward.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kS Static friction feedforward gain.
     * @param kV Velocity feedforward gain.
     */
    public RotationalPIDFController(double kP, double kI, double kD, double kS, double kV) {
        this(kP, kI, kD, kS, kV, 0);
    }

    /**
     * Full constructor for a rotational PIDF controller with all feedforward terms.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kS Static friction feedforward gain.
     * @param kV Velocity feedforward gain.
     * @param kG Gravity feedforward gain.
     */
    public RotationalPIDFController(double kP, double kI, double kD, double kS, double kV, double kG) {
        super(kP, kI, kD, kS, kV);
        this.kG = kG;
    }

    /**
     * Computes the controller output using the current and target positions, and the current angle for gravity FF.
     *
     * @param currentPosition The current position/angle of the mechanism.
     * @param targetPosition  The target position/angle of the mechanism.
     * @param currentAngleRad The current angle of the mechanism in radians, for gravity compensation.
     * @return The computed controller output.
     */
    public double compute(double currentPosition, double targetPosition, double currentAngleRad) {
        this.currentAngleRad = currentAngleRad;
        return super.compute(currentPosition, targetPosition);
    }

    /**
     * Computes the controller output with an explicit velocity measurement.
     *
     * @param currentPosition The current position/angle of the mechanism.
     * @param targetPosition  The target position/angle of the mechanism.
     * @param measuredVelocity The externally measured velocity.
     * @param currentAngleRad The current angle in radians for gravity compensation.
     * @return The computed controller output.
     */
    public double computeWithVelocity(double currentPosition, double targetPosition, double measuredVelocity, double currentAngleRad) {
        this.currentAngleRad = currentAngleRad;
        return super.computeWithVelocity(currentPosition, targetPosition, measuredVelocity);
    }

    /**
     * Overrides the feedforward calculation to include the gravity term.
     *
     * @param velocity The current velocity.
     * @param acceleration The target acceleration.
     * @return The calculated feedforward value.
     */
    @Override
    protected double calculateFeedforward(double velocity, double acceleration) {
        double baseFF = super.calculateFeedforward(velocity, acceleration);
        return baseFF + kG * Math.cos(currentAngleRad);
    }

    /**
     * Sets the feedforward gains, including the gravity gain.
     *
     * @param kS Static friction gain.
     * @param kV Velocity feedforward gain.
     * @param kG Gravity feedforward gain.
     */
    public void setFeedforward(double kS, double kV, double kG) {
        super.setFeedforward(kS, kV);
        this.kG = kG;
    }

    /**
     * Sets the feedforward gains, including acceleration and gravity gains.
     *
     * @param kS Static friction gain.
     * @param kV Velocity feedforward gain.
     * @param kA Acceleration feedforward gain.
     * @param kG Gravity feedforward gain.
     */
    public void setFeedforward(double kS, double kV, double kA, double kG) {
        super.setFeedforward(kS, kV, kA);
        this.kG = kG;
    }

    /**
     * @return The gravity feedforward gain.
     */
    public double getkG() {
        return kG;
    }
}

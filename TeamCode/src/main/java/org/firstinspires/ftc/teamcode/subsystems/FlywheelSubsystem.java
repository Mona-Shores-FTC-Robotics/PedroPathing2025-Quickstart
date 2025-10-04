package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * FlywheelSubsystem controls a high‑speed shooter wheel using a combination
 * of bang‑bang, feedforward and proportional control.  The control logic
 * is run once per loop via {@link #controlLoopOnce()} and does not start its
 * own thread.
 *
 * <p>Use {@link #setTargetRpm(double)} to set the desired wheel speed.
 * Calling {@link #stop()} will set the target to zero and power down the motor.</p>
 */
public class FlywheelSubsystem {

    private final DcMotorEx motor;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastPosTicks = 0.0;
    private double lastRpm = 0.0;
    private double targetRpm = 0.0;

    /**
     * Construct a FlywheelSubsystem.  The motor is looked up by the name
     * "flywheel" in the hardware map.  Change this if your configuration uses
     * a different name.
     */
    public FlywheelSubsystem(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "flywheel");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        loopTimer.reset();
    }

    /** Set the desired RPM for the flywheel.  Passing a value ≤ 0 stops the wheel. */
    public void setTargetRpm(double rpm) {
        this.targetRpm = Math.max(0.0, rpm);
    }

    /** Returns the current target RPM. */
    public double getTargetRpm() {
        return targetRpm;
    }

    /** Compute and return the current measured RPM using encoder ticks. */
    public double getRpm() {
        double dt = Math.max(1e-3, loopTimer.seconds());
        loopTimer.reset();
        double pos = motor.getCurrentPosition();
        double dTicks = pos - lastPosTicks;
        lastPosTicks = pos;

        // Convert ticks per dt to RPM
        double revs = dTicks / Constants.TICKS_PER_REV;
        double rpm = revs / dt * 60.0;

        // Simple smoothing to reduce measurement noise
        lastRpm = 0.7 * lastRpm + 0.3 * rpm;
        return lastRpm;
    }

    /**
     * Run one iteration of the bang‑bang + feedforward + P control loop.  This
     * should be called once per OpMode loop.  It reads the current speed,
     * computes a new motor power and applies it.
     */
    public void controlLoopOnce() {
        double rpm = getRpm();
        double err = targetRpm - rpm;
        double out;

        if (targetRpm <= 0.0) {
            out = 0.0;
        } else if (rpm < targetRpm - Constants.FLY_BB_THRESH_RPM) {
            // Far below target: slam full power
            out = Constants.FLY_MAX_POWER;
        } else if (rpm > targetRpm + Constants.FLY_BB_THRESH_RPM) {
            // Above target: cut power to let it coast down
            out = 0.0;
        } else {
            // Near target: feedforward plus proportional trim
            double ff = Constants.FLY_KF * targetRpm;
            double p = Constants.FLY_KP * err;
            out = ff + p;
        }

        // Clamp the output to the allowed range and set the motor power
        out = Math.max(0.0, Math.min(Constants.FLY_MAX_POWER, out));
        motor.setPower(out);
    }

    /** Stop the flywheel by setting the target RPM to zero and zeroing power. */
    public void stop() {
        targetRpm = 0.0;
        motor.setPower(0.0);
    }
}

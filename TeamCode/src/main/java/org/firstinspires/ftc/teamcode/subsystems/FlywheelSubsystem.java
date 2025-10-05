package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;
import dev.nextftc.core.subsystems.Subsystem;

public class FlywheelSubsystem implements Subsystem {

    private final DcMotorEx motor;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastPosTicks = 0.0;
    private double lastRpm = 0.0;
    private double targetRpm = 0.0;
    private double lastPower = 0.0;

    public FlywheelSubsystem(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "flywheel");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        loopTimer.reset();
    }

    // ===== Public API used by Commands =====
    public void setTargetRpm(double rpm) { this.targetRpm = Math.max(0.0, rpm); }
    public double getTargetRpm()          { return targetRpm; }
    public double getLastPower()          { return lastPower; }

    /** Returns the current measured RPM using encoder ticks. */
    public double getRpm() {
        double dt = Math.max(1e-3, loopTimer.seconds());
        loopTimer.reset();
        double pos = motor.getCurrentPosition();
        double dTicks = pos - lastPosTicks;
        lastPosTicks = pos;

        double revs = dTicks / Constants.TICKS_PER_REV;
        double rpm = revs / dt * 60.0;

        lastRpm = 0.7 * lastRpm + 0.3 * rpm; // EMA smoothing
        return lastRpm;
    }

    public void stop() {
        targetRpm = 0.0;
        motor.setPower(0.0);
    }

    // ===== Subsystem lifecycle (NextFTC scheduler calls this) =====
    @Override public void periodic() {
        double rpm = getRpm();
        double err = targetRpm - rpm;
        double out;

        if (targetRpm <= 0.0) {
            out = 0.0;
        } else if (rpm < targetRpm - Constants.FLY_BB_THRESH_RPM) {
            out = Constants.FLY_MAX_POWER;               // bang-bang: boost up
        } else if (rpm > targetRpm + Constants.FLY_BB_THRESH_RPM) {
            out = 0.0;                                   // bang-bang: coast down
        } else {
            double ff = Constants.FLY_KF * targetRpm;    // feedforward
            double p  = Constants.FLY_KP * err;          // proportional trim
            out = ff + p;
        }

        out = Math.max(0.0, Math.min(Constants.FLY_MAX_POWER, out));
        motor.setPower(out);
        lastPower = out;
    }
}

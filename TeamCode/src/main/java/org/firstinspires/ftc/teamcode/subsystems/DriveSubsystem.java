package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotState;

public class DriveSubsystem implements Subsystem {

    private final Follower follower;
    private Pose teleopSeedPose = null;

    private boolean slowMode = false;
    private double  slowMult = 0.5;
    private boolean robotCentric = false;

    public DriveSubsystem(HardwareMap hw) {
        this.follower = Constants.createFollower(hw);
    }

    /** Optional: let TeleOp provide a seed before init. */
    public void setTeleopSeedPose(Pose p) { this.teleopSeedPose = p; }

    @Override
    public void initialize() {
        // Ensure we’re in manual mode if anything was running
        if (follower.isBusy()) {
            follower.startTeleopDrive();
        }

        // Prefer Auto→TeleOp handoff; else TeleOp seed; else default (0,0,0)
        Pose seed = RobotState.takeHandoffPose();
        if (seed == null) seed = (teleopSeedPose != null) ? teleopSeedPose : new Pose();

        follower.setStartingPose(seed);
        follower.update();            // warm-up tick
        follower.startTeleopDrive();  // set motor modes per Pedro config
    }

    @Override
    public void periodic() {
        follower.update();            // MUST run every loop
    }

    /** Call this from your OpMode's onStop()/stop(). */
    public void shutdown() {
        follower.startTeleopDrive();  // ensure manual mode
        drive(0, 0, 0);               // zero inputs so motors settle
    }

    // --- TeleOp driving (field- or robot-centric) ---
    public void drive(double lx, double ly, double rx) {
        double s = slowMode ? slowMult : 1.0;
        follower.setTeleOpDrive(
                +ly * s,   // forward (+)
                -lx * s,   // strafeRight (our lx is +left)
                -rx * s,   // turnRight  (our rx is +CCW)
                robotCentric
        );
    }

    public void setRobotCentric(boolean rc) { this.robotCentric = rc; }
    public boolean isRobotCentric()         { return robotCentric;   }
    public void setSlowMode(boolean b)      { slowMode = b; }
    public boolean isSlowMode()             { return slowMode; }
    public void setSlowMult(double m)       { slowMult = Math.max(0.1, Math.min(1.0, m)); }

    // In DriveSubsystem

    // Simple DTO you already defined:
    public static class Pose2d {
        public double x, y, headingRad;
        public Pose2d(double x, double y, double h) { this.x = x; this.y = y; this.headingRad = h; }
    }

    /** Current field pose from Pedro (x,y in Pedro units, heading in radians). */
    public Pose2d getPose() {
        Pose p = follower.getPose();                 // Pedro pose
        return new Pose2d(p.getX(), p.getY(), follower.getHeading());
    }

    /** Convenience getters if you like */
    public double getX()          { return follower.getPose().getX(); }
    public double getY()          { return follower.getPose().getY(); }
    public double getHeadingRad() { return follower.getHeading();     }

    /** Optional: setter you already had */
    public void setPose(double x, double y, double headingRad) {
        follower.setStartingPose(new Pose(x, y, headingRad));
        follower.startTeleopDrive();
    }

}

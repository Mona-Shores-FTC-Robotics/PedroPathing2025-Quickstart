package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotState;

public class DriveSubsystem implements Subsystem {

    public enum DriveMode { NORMAL, SLOW }

    private final Follower follower;

    // Keep this simple to mirror LocalizationTest
    public boolean poseIsMeters   = false;
    public double  slowMultiplier = 0.20;     // Only used when SLOW is active

    private boolean robotCentric  = false;     // LocalizationTest passes true
    private DriveMode defaultMode = DriveMode.NORMAL;
    private DriveMode activeMode  = DriveMode.NORMAL;

    private Pose teleopSeedPose = null;

    public DriveSubsystem(HardwareMap hw) {
        this.follower = Constants.createFollower(hw);
    }

    public void setTeleopSeedPose(Pose p) { this.teleopSeedPose = p; }
    public void setDefaultMode(DriveMode mode) { this.defaultMode = mode; this.activeMode = mode; }
    public void setRobotCentric(boolean rc) { this.robotCentric = rc; }
    public boolean isRobotCentric() { return robotCentric; }

    @Override
    public void initialize() {
        if (follower.isBusy()) follower.breakFollowing();

        Pose seed = RobotState.takeHandoffPose();
        if (seed == null) seed = (teleopSeedPose != null) ? teleopSeedPose : new Pose();

        follower.setStartingPose(seed);
        follower.update();            // warmup
        follower.startTeleopDrive();  // same as LocalizationTest.start()
    }

    @Override
    public void periodic() {
        follower.update();            // same as LocalizationTest.loop()
    }

    public void shutdown() {
        // Match Tuning.stopRobot behavior for safety on stop
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    /**
     * Minimal drive, mirrors LocalizationTest mapping:
     * LocalizationTest uses: setTeleOpDrive(-left_y, -left_x, -right_x, true)
     *
     * @param lx left stick X  (+left from driver)
     * @param ly left stick Y  (+forward from driver)
     * @param rx right stick X (+CCW from driver convention)
     * @param rightBumperHeld enable SLOW when true
     * @param leftBumperHeld  force NORMAL when true
     */
    public void driveWithModeHolds(double lx, double ly, double rx,
                                   boolean rightBumperHeld, boolean leftBumperHeld) {

        if (rightBumperHeld)      activeMode = DriveMode.SLOW;
        else if (leftBumperHeld)  activeMode = DriveMode.NORMAL;
        else                      activeMode = defaultMode;

        double mult = (activeMode == DriveMode.SLOW) ? slowMultiplier : 1.0;

        // Exact sign pattern used by LocalizationTest
        double forward     = ly * mult;
        double strafeLeft  = -lx * mult;
        double turnRight   = rx * mult;   // driver rx is CCW+, Pedro expects +turnRight

        follower.setTeleOpDrive(forward, strafeLeft, turnRight, robotCentric);
    }

    // Convenience API if you want a direct passthrough like LocalizationTest
    public void driveSimple(double lx, double ly, double rx) {
        follower.setTeleOpDrive(-ly, -lx, -rx, robotCentric);
    }

    // Telemetry helpers
    public DriveMode getActiveMode() { return activeMode; }

    public Pose2D getPose() {
        Pose p = follower.getPose();
        DistanceUnit du = poseIsMeters ? DistanceUnit.METER : DistanceUnit.INCH;
        return new Pose2D(du, p.getX(), p.getY(), AngleUnit.RADIANS, follower.getHeading());
    }

    public double getX()          { return follower.getPose().getX(); }
    public double getY()          { return follower.getPose().getY(); }
    public double getHeadingRad() { return follower.getHeading();     }

    public void setPose(double x, double y, double headingRad) {
        follower.setStartingPose(new Pose(x, y, headingRad));
        follower.startTeleopDrive();
    }
}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.DoubleSupplier;

/**
 * Thin wrapper around Pedro Follower so TeleOp doesn't touch hardware directly.
 * Responsibilities:
 *  - Own the Follower
 *  - Provide pose/heading accessors
 *  - Offer a field-centric drive method
 *  - Allow swapping the yaw source (e.g., Pinpoint)
 */
public class Drivetrain {

    private final Follower follower;
    private DoubleSupplier yawDegSupplier; // default: follower heading → degrees

    public Drivetrain(HardwareMap hw, Pose startingPose) {
        follower = Constants.createFollower(hw);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        // Default yaw provider: follower heading in radians → degrees
        yawDegSupplier = () -> Math.toDegrees(follower.getHeading());
    }

    /** Call each loop before using pose/drive. */
    public void update() {
        follower.update();
    }

    /** Call once at TeleOp start. */
    public void startTeleop() {
        follower.startTeleopDrive();
    }

    /** Field-centric drive passthrough. */
    public void driveField(double fx, double fy, double omegaRad, boolean fieldOrRobotFlag) {
        follower.setTeleOpDrive(fx, fy, omegaRad, fieldOrRobotFlag);
    }

    /** Current field pose (Pedro frame: X forward from alliance wall, Y left). */
    public Pose getPose() {
        return follower.getPose();
    }

    /** Current robot yaw (field frame) from the chosen provider, in RADIANS. */
    public double getYawRad() {
        return Math.toRadians(yawDegSupplier.getAsDouble());
    }

    /** Swap in Pinpoint or another yaw provider (expects DEGREES). */
    public void setYawProviderDegrees(DoubleSupplier yawDegrees) {
        this.yawDegSupplier = yawDegrees;
    }
}

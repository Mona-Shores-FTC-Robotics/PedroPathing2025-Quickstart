package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * DriveSubsystem provides field‑centric mecanum control with optional
 * auto‑heading and a slow mode toggle.  It also maintains a simple pose
 * estimate for telemetry and simulation via DevSim.
 *
 * <p>The subsystem uses the IMU (if present) for heading feedback and maps
 * joystick inputs from the field frame to the robot frame.  When not in slow
 * mode it automatically rotates the robot to face the direction of motion
 * (determined from the left stick).  In slow mode the right stick controls
 * rotation directly and auto‑heading is disabled.</p>
 */
public class DriveSubsystem {

    /**
     * Simple pose representation in inches and radians.  Used for telemetry.
     */
    public static class Pose2d {
        public double x;        // inches
        public double y;        // inches
        public double headingRad; // radians (CCW)

        public Pose2d(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.headingRad = h;
        }
    }

    private final DcMotorEx fl;
    private final DcMotorEx fr;
    private final DcMotorEx bl;
    private final DcMotorEx br;
    //
    private BNO055IMU imu;
    private boolean slowMode = false;
    private Pose2d pose = new Pose2d(0, 0, 0);
    private final ElapsedTime simTimer = new ElapsedTime();

    /**
     * Construct a DriveSubsystem from the provided hardware map.  Motor names
     * are taken from {@link Constants}.  If an IMU with name
     * {@link Constants#IMU_NAME} exists it will be used for heading.
     */
    public DriveSubsystem(HardwareMap hw) {
        fl = hw.get(DcMotorEx.class, Constants.MOTOR_FRONT_LEFT);
        fr = hw.get(DcMotorEx.class, Constants.MOTOR_FRONT_RIGHT);
        bl = hw.get(DcMotorEx.class, Constants.MOTOR_BACK_LEFT);
        br = hw.get(DcMotorEx.class, Constants.MOTOR_BACK_RIGHT);

        // Reverse left side motors so that positive power drives forward.
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.FORWARD);

        // Brake when zeroed helps precise control
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Attempt to obtain an IMU; swallow any exceptions if not present
        try {
            if (Constants.IMU_NAME != null) {
                imu = hw.get(BNO055IMU.class, Constants.IMU_NAME);
                BNO055IMU.Parameters params = new BNO055IMU.Parameters();
                params.mode = BNO055IMU.SensorMode.IMU;
                params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(params);
            }
        } catch (Exception ignored) {
            imu = null;
        }

        simTimer.reset();
    }

    /** Enable or disable slow mode.  When enabled all inputs are scaled and
     * auto‑heading is suppressed. */
    public void setSlowMode(boolean enabled) {
        this.slowMode = enabled;
    }

    /** Returns whether slow mode is currently enabled. */
    public boolean isSlowMode() {
        return slowMode;
    }

    /** Returns the current estimated pose in inches and radians. */
    public Pose2d getPose() {
        return pose;
    }

    /** Sets the current pose estimate.  Used to reset position in auto. */
    public void setPose(double x, double y, double headingRad) {
        pose.x = x;
        pose.y = y;
        pose.headingRad = headingRad;
    }

    /**
     * Drive the robot based on field‑centric joystick inputs.  The left stick
     * commands translation (lx = leftwards, ly = forwards) in the field
     * frame.  The right stick commands rotation around the vertical axis.  If
     * slow mode is disabled and the left stick magnitude is above a small
     * threshold the robot will automatically rotate toward the direction of
     * motion.
     *
     * @param lx horizontal component of the left stick (left is positive)
     * @param ly vertical component of the left stick (forward is positive)
     * @param rx horizontal component of the right stick (CCW rotation is positive)
     */
    public void drive(double lx, double ly, double rx) {
        // Apply slow mode scaling
        double scale = slowMode ? Constants.SLOW_MODE_SCALE : 1.0;
        lx *= scale;
        ly *= scale;
        rx *= scale;

        // Determine current heading (prefer IMU if available)
        double heading = pose.headingRad;
        if (imu != null) {
            heading = imu.getAngularOrientation().firstAngle;
        }

        // Convert field frame (left, forward) to robot frame (xRobot = left, yRobot = forward)
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double xRobot = lx * cos - ly * sin;
        double yRobot = lx * sin + ly * cos;

        // Auto‑heading: drive rotates to face the direction of travel when not in slow mode
        double rotCmd = rx;
        double mag = Math.hypot(lx, ly);
        if (!slowMode && mag > 0.08) {
            double desiredHeading = Math.atan2(lx, ly); // field direction
            double err = wrap(desiredHeading - heading);
            double errDeg = Math.toDegrees(err);
            if (Math.abs(errDeg) > Constants.HEADING_DEADBAND_DEG) {
                rotCmd += Constants.HEADING_KP * err;
            }
        }

        // Mecanum inverse kinematics (robot frame)
        double flP = yRobot + xRobot + rotCmd;
        double frP = yRobot - xRobot - rotCmd;
        double blP = yRobot - xRobot + rotCmd;
        double brP = yRobot + xRobot - rotCmd;

        // Normalise to keep within ±1
        double max = Math.max(1.0, Math.max(Math.abs(flP),
                Math.max(Math.abs(frP), Math.max(Math.abs(blP), Math.abs(brP)))));
        flP /= max;
        frP /= max;
        blP /= max;
        brP /= max;

        // Clip to motor power limits and apply
        fl.setPower(Range.clip(flP, -Constants.MAX_DRIVE_POWER, Constants.MAX_DRIVE_POWER));
        fr.setPower(Range.clip(frP, -Constants.MAX_DRIVE_POWER, Constants.MAX_DRIVE_POWER));
        bl.setPower(Range.clip(blP, -Constants.MAX_DRIVE_POWER, Constants.MAX_DRIVE_POWER));
        br.setPower(Range.clip(brP, -Constants.MAX_DRIVE_POWER, Constants.MAX_DRIVE_POWER));

        // Update pose with DevSim if enabled.  When real odometry is present
        // replace this method with your own integration and disable DevSim.
        updateOdometryDevSim(lx, ly, rotCmd);
    }

    /** Simple integrator for DevSim.  Uses constant velocities derived from
     * joystick inputs to update the pose when no real odometry is available. */
    private void updateOdometryDevSim(double lx, double ly, double rotCmd) {
        if (!Constants.DEV_SIM_ENABLED) {
            return;
        }

        double dt = simTimer.seconds();
        simTimer.reset();

        // Map field stick inputs to chassis speeds (inches per second)
        double vx = ly * Constants.DEV_SIM_SPEED_IPS; // forward
        double vy = lx * Constants.DEV_SIM_SPEED_IPS; // left
        double omega = rotCmd * Math.toRadians(Constants.DEV_SIM_TURN_DPS); // rad/s

        double h = pose.headingRad;
        double dx = (vx * Math.cos(h) - vy * Math.sin(h)) * dt;
        double dy = (vx * Math.sin(h) + vy * Math.cos(h)) * dt;
        double dh = omega * dt;

        pose.x += dx;
        pose.y += dy;
        pose.headingRad = wrap(h + dh);
    }

    private static double wrap(double a) {
        while (a > Math.PI) {
            a -= 2.0 * Math.PI;
        }
        while (a < -Math.PI) {
            a += 2.0 * Math.PI;
        }
        return a;
    }
}

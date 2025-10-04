package org.firstinspires.ftc.teamcode;

/**
 * Central configuration and tuning constants for the command‑based DECODE robot.
 *
 * <p>This class centralises hardware names, drivetrain geometry, flywheel
 * control gains and DevSim toggles. The goal is to have a single place where
 * important numbers live so that students can quickly understand and adjust
 * how the robot behaves.</p>
 */
import com.acmerobotics.dashboard.config.Config;

/**
 * Central configuration and tuning constants for the command‑based DECODE robot.
 *
 * <p>This class centralises hardware names, drivetrain geometry, flywheel
 * control gains and DevSim toggles.  Marking the class with {@link Config}
 * exposes the public static fields to the FTC Dashboard’s config UI.  This
 * allows you to tune parameters on the fly without redeploying code.  Only
 * non‑final public static fields will show up as editable entries.</p>
 */
@Config
public final class Constants {
    private Constants() {}

    /** Hardware map names for the drivetrain.  These are not expected to change. */
    public static final String MOTOR_FRONT_LEFT  = "lf";
    public static final String MOTOR_FRONT_RIGHT = "rf";
    public static final String MOTOR_BACK_LEFT   = "lb";
    public static final String MOTOR_BACK_RIGHT  = "rb";
    /** Optional IMU name. Set to null if not used. */
    public static final String IMU_NAME = "imu";

    // === Drive geometry and control ===
    /** Wheel radius in inches used for simple odometry. */
    public static final double WHEEL_RADIUS_IN = 2.0;
    /** Encoder ticks per wheel revolution.  This is typically fixed by the motor/gearbox. */
    public static final double TICKS_PER_REV   = 537.7;
    /** Maximum drive power sent to the motors.  Keep as a constant to prevent over‑driving. */
    public static final double MAX_DRIVE_POWER = 1.0;

    /** Heading hold proportional gain used by {@link org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem}. */
    public static double HEADING_KP = 2.0;
    /** Deadband on heading error (degrees) below which no correction is applied. */
    public static double HEADING_DEADBAND_DEG = 6.0;

    /** Scale applied to all inputs when the driver holds the slow mode button. */
    public static double SLOW_MODE_SCALE = 0.35;

    // === Flywheel control ===
    /**
     * Feedforward constant (unitless).  Multiplies the target RPM to produce a base
     * motor power.  Tune this on the real robot by measuring the RPM at full
     * power and computing 1/maxRPM.  Exposed for live tuning.
     */
    public static double FLY_KF = 0.00013;
    /** Proportional gain for fine adjustments around the target flywheel RPM. */
    public static double FLY_KP = 0.0006;
    /** Bang‑bang threshold in RPM.  When the actual speed is further than this
     *  from the setpoint the controller will apply full power or zero power. */
    public static double FLY_BB_THRESH_RPM = 150.0;
    /** Maximum power applied to the flywheel motor. */
    public static double FLY_MAX_POWER = 1.0;
    /** Convenient default RPM for testing the flywheel. */
    public static double FLY_TEST_RPM = 5000.0;

    // === DevSim ===
    /**
     * When set to true the DriveSubsystem will integrate joystick inputs to
     * update its pose without any hardware.  This allows you to see the robot
     * move on the FTC Dashboard and AdvantageScope even when no motors are
     * connected.  Set this to false when running on an actual robot with
     * working odometry.
     */
    public static boolean DEV_SIM_ENABLED = true;
    /** Simulated linear speed (inches per second) used by DevSim for drive. */
    public static double  DEV_SIM_SPEED_IPS = 24.0;
    /** Simulated turn speed (degrees per second) used by DevSim for heading. */
    public static double  DEV_SIM_TURN_DPS  = 120.0;

    /**
     * Placeholder factory for a Pedro follower.  This method will be replaced
     * once Pedro/NextFTC integration is added.  For now it throws an
     * exception so that calls to it are obvious errors.
     */
    public static Object createPedroFollower() {
        throw new UnsupportedOperationException("Pedro follower not yet integrated");
    }
}

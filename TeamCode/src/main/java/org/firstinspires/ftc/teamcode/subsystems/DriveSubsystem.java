package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.VisionCalibrationState.HEADING_KNOWN;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.VisionCalibrationState.HEADING_UNKNOWN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveAimAssistConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveFixedAngleAimConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveFusionConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveInitialPoseConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveRampConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveRightTriggerFixedAngleConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveTeleOpConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveVisionRelocalizeConfig;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseFrames;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;

/**
 * Owns the four mecanum drive motors and the Pedro Pathing follower.
 *
 * <p>What it does:
 * <ul>
 *   <li><b>Teleop driving</b> — translates joystick inputs into wheel commands,
 *       in field-centric mode by default (push the stick "north" and the robot
 *       moves toward the far wall regardless of which way it's facing).</li>
 *   <li><b>Aim assist</b> — when an aim button is held, takes over rotation and
 *       points the robot at the goal using vision + odometry.</li>
 *   <li><b>Autonomous path following</b> — exposes the Pedro {@code Follower} so
 *       OpModes can run pre-built paths.</li>
 *   <li><b>Relocalization</b> — fuses Pinpoint odometry with AprilTag readings
 *       so the robot's reported position stays accurate even after bumps.</li>
 * </ul>
 *
 * <p>Per-robot tuning (PIDF gains, aim parameters, etc.) lives in
 * {@code util/RobotProfile.java} — never branch on robot identity here.
 */
@Configurable
public class DriveSubsystem {

    /** Robot is considered "stationary enough to shoot" when its speed is below this (inches/sec). */
    public static final double STATIONARY_SPEED_THRESHOLD_IN_PER_SEC = 1.5;
    /** Ignore an AprilTag snapshot for relocalization once it's older than this. */
    private static final long MAX_VISION_AGE_MS = 250L;

    enum VisionCalibrationState {
        HEADING_UNKNOWN,
        HEADING_KNOWN
    }
    private VisionCalibrationState visionState = HEADING_UNKNOWN;
    private int consecutiveMt2DisagreementCount = 0;

    // Global configuration instances
    public static DriveTeleOpConfig teleOpDriveConfig = new DriveTeleOpConfig();
    public static DriveRampConfig rampConfig = new DriveRampConfig();
    public static DriveVisionRelocalizeConfig visionRelocalizeConfig = new DriveVisionRelocalizeConfig();


    /** Live aim-assist tuning for the active robot — see {@code util/RobotProfile.java} for the values. */
    public static DriveAimAssistConfig aimAssistConfig =
            org.firstinspires.ftc.teamcode.util.RobotProfile.forCurrent().aimAssist;

    /** Live fixed-angle aim tuning for the active robot — see {@code util/RobotProfile.java} for the values. */
    public static DriveFixedAngleAimConfig fixedAngleAimConfig =
            org.firstinspires.ftc.teamcode.util.RobotProfile.forCurrent().fixedAngleAim;

    /** Live right-trigger fixed-angle aim tuning for the active robot — see {@code util/RobotProfile.java}. */
    public static DriveRightTriggerFixedAngleConfig rightTriggerFixedAngleConfig =
            org.firstinspires.ftc.teamcode.util.RobotProfile.forCurrent().rightTriggerFixedAngle;

    /**
     * Re-binds the per-robot static config fields from {@code RobotProfile}.
     *
     * <p>The field initializers above run during the boot-time {@code @Configurable}
     * scan, before the robot is identified, so they capture the 20245 fallback. Call
     * this after {@code RobotProfile.invalidate()} once the SSID is known. The instance
     * fields used for drive behavior come from the constructor, but these statics back
     * the Panels tuning surface — refreshing them keeps Panels edits pointed at the same
     * config objects the running subsystem uses.
     */
    public static void reloadProfileConfigs() {
        aimAssistConfig = org.firstinspires.ftc.teamcode.util.RobotProfile.forCurrent().aimAssist;
        fixedAngleAimConfig = org.firstinspires.ftc.teamcode.util.RobotProfile.forCurrent().fixedAngleAim;
        rightTriggerFixedAngleConfig = org.firstinspires.ftc.teamcode.util.RobotProfile.forCurrent().rightTriggerFixedAngle;
    }

    /**
     * Gets the robot-specific InitialPoseConfig based on RobotState.getRobotName().
     * @return initialPoseConfig19429 or initialPoseConfig20245
     */
    public static DriveInitialPoseConfig initialPoseConfig = new DriveInitialPoseConfig();

    public String visionRelocalizeStatus = "Press A to re-localize";
    public long visionRelocalizeStatusMs = 0L;
    private String lastRelocalizeSource = "none";

    public enum DriveMode {
        NORMAL,
        SLOW
    }

    // Removed NORMAL_MULTIPLIER constant - now uses teleOpDriveConfig.normalMultiplier

    private Follower follower;
    private final Constants.Motors driveMotors;
    private final DcMotorEx motorLf;
    private final DcMotorEx motorRf;
    private final DcMotorEx motorLb;
    private final DcMotorEx motorRb;
    private final VisionSubsystemLimelight vision;
    private LightingSubsystem lighting;
    private FusionLocalizer fusionLocalizer;
    private long lastFedSnapshotNs = -1L;
    private final ElapsedTime clock = new ElapsedTime();

    public static boolean robotCentricConfig = false;
    private boolean robotCentric = robotCentricConfig;
    private DriveMode activeMode = DriveMode.NORMAL;
    private double lastRequestFieldX = 0.0;
    private double lastRequestFieldY = 0.0;
    private double lastRequestRotation = 0.0;
    private boolean lastRequestSlowMode = false;
    private double lastCommandForward = 0.0;
    private double lastCommandStrafeLeft = 0.0;
    private double lastCommandTurn = 0.0;
    private double lastAimErrorRad = Double.NaN;
    private long lastAimUpdateNs = 0L;
    private double lastAimTurnCommand = 0.0;
    private boolean rampModeActive = false;
    private double rampForward = 0.0;
    private double rampStrafeLeft = 0.0;
    private double rampTurn = 0.0;
    private long lastRampUpdateNs = 0L;
    private boolean teleOpControlEnabled = false;
    private boolean visionRelocalizationEnabled = false;
    private long lastAutoRelocalizeMs = 0L;

    /** Per-robot aim assist tuning, injected at construction. */
    private final DriveAimAssistConfig aimAssist;
    /** Per-robot fixed-angle aim tuning, injected at construction. */
    private final DriveFixedAngleAimConfig fixedAngleAim;
    /** Per-robot right-trigger fixed-angle aim tuning, injected at construction. */
    private final DriveRightTriggerFixedAngleConfig rightTriggerFixedAngle;

    public DriveSubsystem(HardwareMap hardwareMap,
                          VisionSubsystemLimelight vision,
                          DriveAimAssistConfig aimAssist,
                          DriveFixedAngleAimConfig fixedAngleAim,
                          DriveRightTriggerFixedAngleConfig rightTriggerFixedAngle) {
        driveMotors = new Constants.Motors(hardwareMap);
        driveMotors.setRunWithoutEncoder();

        motorLf = driveMotors.lf;
        motorRf = driveMotors.rf;
        motorLb = driveMotors.lb;
        motorRb = driveMotors.rb;
        this.vision = vision;
        this.aimAssist = aimAssist;
        this.fixedAngleAim = fixedAngleAim;
        this.rightTriggerFixedAngle = rightTriggerFixedAngle;
    }

    public void setLightingSubsystem(LightingSubsystem lighting) {
        this.lighting = lighting;
    }

    public void attachFollower() {
        this.follower = Constants.follower();
        this.fusionLocalizer = Constants.activeFusionLocalizer;
        // Log which config set is being used for diagnostics
        if (TelemetrySettings.isVerbose()) {
            RobotState.packet.put("_Config/Active Config Set", org.firstinspires.ftc.teamcode.util.RobotProfile.activeName());
        }
    }
//comment
    public void setRobotCentric(boolean enabled) {
        robotCentric = enabled;
        robotCentricConfig = enabled;
    }

    public void setTeleOpControlEnabled(boolean enabled) {
        teleOpControlEnabled = enabled;
    }

    public void initialize() {
        // Defensive check: ensure follower is attached before initialization
        if (follower == null) {
            throw new IllegalStateException(
                "DriveSubsystem: Follower not attached! Call attachFollower() before initialize(). " +
                "This typically happens in Robot.attachPedroFollower() during OpMode init.");
        }

        if (follower.isBusy()) {
            follower.breakFollowing();
        }

        Pose pedroFollowerSeed = RobotState.takeHandoffPose();

        if (pedroFollowerSeed == null) {
            DriveInitialPoseConfig poseConfig = initialPoseConfig;
            pedroFollowerSeed = new Pose(poseConfig.startX, poseConfig.startY, Math.toRadians(poseConfig.startHeadingDeg));
        }
        Pose ftcSeed = PoseFrames.pedroToFtc(pedroFollowerSeed);
        RobotState.putPose("Pose/FTC Coord Seed Pose",ftcSeed );

        // Follower initialization
        follower.setStartingPose(pedroFollowerSeed);
        follower.setPose(pedroFollowerSeed);
        follower.update();
        follower.breakFollowing();

        visionRelocalizationEnabled = true;
    }

    /**
     * Starts teleop drive mode, enabling motor control.
     * Should be called when the match starts (after init phase).
     */
    public void startTeleopDrive() {
        if (teleOpControlEnabled) {
            follower.startTeleopDrive();
        }
    }

    private double lastPeriodicMs = 0.0;

    /** Nanosecond timestamp when the follower most recently became busy; -1 when idle. */
    private long pathBusyStartNs = -1L;
    /** Which completion term last kept the follower busy when a path finished (for post-mortem). */
    private String lastPathHoldout = "none";

    // Snapshot of the last active-path frame, used to log a completion line on the busy→idle edge.
    private boolean pathWasBusy = false;
    private double lastHeadingErrDeg = 0.0;
    private double lastHeadingAngVelDps = 0.0;
    private double lastTransErrIn = 0.0;
    private double lastVelIps = 0.0;
    private double lastBusyMs = 0.0;
    /** Accumulating per-run log: one line per completed path, copy/pasteable from AdvantageScope. */
    private final StringBuilder completionLog = new StringBuilder();
    private int completionCount = 0;

    /**
     * Infinite Command that drives the Pedro follower update + vision feed each scheduler tick.
     * Scheduled once in OpMode init; runs until OpMode stop.
     */
    public com.pedropathing.ivy.Command periodic() {
        return Commands.infinite(this::doPeriodic);
    }

    private void doPeriodic() {
        long start = System.nanoTime();
        follower.update();
        lastPeriodicMs = (System.nanoTime() - start) / 1_000_000.0;

        // MegaTag2: Update vision subsystem with current heading for IMU-fused localization
        // This must happen before vision.periodic() processes AprilTag detections
        if (vision != null) {
            double headingRad = follower.getHeading();
            vision.setRobotHeading(headingRad);
        }

        maybeFeedVisionMeasurement();
        if (TelemetrySettings.isVerbose()) {
            RobotState.packet.put("/vision/state", visionState.name());
        }

        // Heading diagnostics for waggle investigation (follower target vs current)
        Pose pose = follower.getPose();
        if (pose != null) {
            // Pedro follower doesn't expose target heading directly; approximate using current path heading
            double currentHeading = follower.getHeading();
            if (TelemetrySettings.isVerbose()) {
                RobotState.packet.put("HeadingDiag/current_deg", Math.toDegrees(currentHeading));
                RobotState.packet.put("HeadingDiag/mode", follower.isBusy() ? "path_follow" : "idle/hold");
                RobotState.packet.put("HeadingDiag/speed_ips", getRobotSpeedInchesPerSecond());
            }
        }

        logPathDiagnostics();

        double lfA = getLfCurrentAmps();
        double rfA = getRfCurrentAmps();
        double lbA = getLbCurrentAmps();
        double rbA = getRbCurrentAmps();
        RobotState.packet.put("drive/lf/current_amps", lfA);
        RobotState.packet.put("drive/rf/current_amps", rfA);
        RobotState.packet.put("drive/lb/current_amps", lbA);
        RobotState.packet.put("drive/rb/current_amps", rbA);
        RobotState.packet.put("drive/total/current_amps", sumCurrentAmps(lfA, rfA, lbA, rbA));
    }

    public double getLastPeriodicMs() {
        return lastPeriodicMs;
    }

    /**
     * End-of-path diagnostics: answers "why is the follower still busy?" by logging each
     * path-completion error term next to the {@link com.pedropathing.paths.PathConstraints}
     * threshold it must drop under, plus how long the current path has been busy.
     *
     * <p>A path reports done ({@code isBusy()} → false) only when t-value, translational
     * error, heading error, and velocity are all under their constraints — or the timeout
     * settle window elapses. {@code PathDiag/holdout} names whichever term is currently the
     * blocker, and {@code PathDiag/last_holdout} latches it at the moment the path finishes,
     * so a stall shows up as a concrete cause rather than a guess. VERBOSE-only.
     */
    private void logPathDiagnostics() {
        boolean busy = follower.isBusy();

        // Track busy duration across loops regardless of telemetry level (cheap).
        long nowNs = System.nanoTime();
        if (busy && pathBusyStartNs < 0) {
            pathBusyStartNs = nowNs;
        }

        if (!TelemetrySettings.isVerbose()) {
            if (!busy) {
                pathBusyStartNs = -1L;
            }
            return;
        }

        // The follower's error getters dereference the active path's ErrorCalculator, which is
        // null when no path is loaded (idle between segments / teleop). Reading them then throws
        // an NPE deep in Follower#getHeadingError, so bail out to a minimal idle report.
        if (!busy || follower.getCurrentPath() == null) {
            pathBusyStartNs = -1L;
            // Busy→idle edge: a path just finished. Append its completion snapshot to the log.
            if (pathWasBusy) {
                pathWasBusy = false;
                completionCount++;
                String line = String.format(
                        "#%d hErr=%.1f angVel=%.1f tErr=%.2f vel=%.2f busy=%.0f hold=%s",
                        completionCount, lastHeadingErrDeg, lastHeadingAngVelDps,
                        lastTransErrIn, lastVelIps, lastBusyMs, lastPathHoldout);
                completionLog.append(line).append("\n");
                // Mirror to logcat (grep marker "PDIAG|") so it can be pulled over adb without
                // copying out of AdvantageScope. VERBOSE-only: this branch is already gated above.
                RobotLog.ii("PathDiag", "PDIAG| " + line);
            }
            RobotState.packet.put("PathDiag/busy", false);
            RobotState.packet.put("PathDiag/busy_ms", 0.0);
            RobotState.packet.put("PathDiag/holdout", "none");
            RobotState.packet.put("PathDiag/last_holdout", lastPathHoldout);
            RobotState.packet.put("PathDiag/completion_log", completionLog.toString());
            // No active path → no target. Publish live actual heading anyway (getHeading is safe
            // without a path) so the heading-track channels are alive during init, with target
            // pinned to actual so the error reads 0 until a path starts driving.
            double idleHeadingDeg = normalizeDeg360(Math.toDegrees(follower.getHeading()));
            RobotState.packet.put("PathDiag/heading_current_deg", idleHeadingDeg);
            RobotState.packet.put("PathDiag/heading_target_deg", idleHeadingDeg);
            RobotState.packet.put("PathDiag/heading_err_deg", 0.0);
            RobotState.packet.put("PathDiag/heading_err_signed_deg", 0.0);
            return;
        }

        com.pedropathing.paths.PathConstraints c = follower.getConstraints();

        double headingErrSignedRad = follower.getHeadingError();
        double headingErrRad = Math.abs(headingErrSignedRad);
        // Target heading the follower is driving toward (path's heading goal at the closest point)
        // vs. where the robot actually is — plot these two against each other in AdvantageScope to
        // see the PID track: overshoot, oscillation, or steady-state offset.
        double headingTargetDeg = normalizeDeg360(Math.toDegrees(follower.getClosestPointHeadingGoal()));
        double headingCurrentDeg = normalizeDeg360(Math.toDegrees(follower.getHeading()));
        Vector transErr = follower.getTranslationalError();
        double transErrIn = transErr != null ? transErr.getMagnitude() : 0.0;
        double velIps = follower.getVelocity() != null ? follower.getVelocity().getMagnitude() : 0.0;
        double tValue = follower.getCurrentTValue();

        // Thresholds (global pathConstraints; per-path heading overrides aren't reflected here).
        double tConstraint = c.getTValueConstraint();
        double transConstraint = c.getTranslationalConstraint();
        double headingConstraintRad = c.getHeadingConstraint();
        double velConstraint = c.getVelocityConstraint();

        RobotState.packet.put("PathDiag/busy", busy);
        RobotState.packet.put("PathDiag/busy_ms", busy ? (nowNs - pathBusyStartNs) / 1_000_000.0 : 0.0);
        RobotState.packet.put("PathDiag/t_value", tValue);
        RobotState.packet.put("PathDiag/t_constraint", tConstraint);
        RobotState.packet.put("PathDiag/heading_target_deg", headingTargetDeg);
        RobotState.packet.put("PathDiag/heading_current_deg", headingCurrentDeg);
        RobotState.packet.put("PathDiag/heading_err_deg", Math.toDegrees(headingErrRad));
        RobotState.packet.put("PathDiag/heading_err_signed_deg", Math.toDegrees(headingErrSignedRad));
        RobotState.packet.put("PathDiag/heading_constraint_deg", Math.toDegrees(headingConstraintRad));
        // Angular velocity (deg/s): near-zero while heading error is still large means the PID
        // has stalled out at a steady-state offset (fix with secondary I); non-zero/swinging
        // means it's still hunting/oscillating (fix with secondary D or lower P).
        RobotState.packet.put("PathDiag/heading_ang_vel_dps", Math.toDegrees(follower.getAngularVelocity()));
        RobotState.packet.put("PathDiag/translational_err_in", transErrIn);
        RobotState.packet.put("PathDiag/translational_constraint_in", transConstraint);
        RobotState.packet.put("PathDiag/drive_err", follower.getDriveError());
        RobotState.packet.put("PathDiag/vel_ips", velIps);
        RobotState.packet.put("PathDiag/vel_constraint", velConstraint);
        RobotState.packet.put("PathDiag/dist_remaining_in", follower.getDistanceRemaining());
        RobotState.packet.put("PathDiag/path_completion", follower.getPathCompletion());

        // List ALL failing completion terms (not just the first). The follower requires every
        // one under tolerance to complete cleanly, so reporting only the first masks the others
        // — e.g. a barely-over translational error hiding a badly-over heading error.
        StringBuilder sb = new StringBuilder();
        if (tValue < tConstraint) sb.append("t_value ");
        if (transErrIn > transConstraint) sb.append("translational ");
        if (headingErrRad > headingConstraintRad) sb.append("heading ");
        if (velIps > velConstraint) sb.append("velocity ");
        // Nothing over tolerance but still busy → burning the timeout settle window.
        String holdout = sb.length() == 0 ? "timeout_settle" : sb.toString().trim();
        RobotState.packet.put("PathDiag/holdout", holdout);

        // Remember the live blocker so it survives the busy→idle transition.
        lastPathHoldout = holdout;
        RobotState.packet.put("PathDiag/last_holdout", lastPathHoldout);

        // Snapshot this frame so the busy→idle edge can log the completion values.
        double busyMs = (nowNs - pathBusyStartNs) / 1_000_000.0;
        lastHeadingErrDeg = Math.toDegrees(headingErrRad);
        lastHeadingAngVelDps = Math.toDegrees(follower.getAngularVelocity());
        lastTransErrIn = transErrIn;
        lastVelIps = velIps;
        lastBusyMs = busyMs;
        pathWasBusy = true;

        // Live one-line summary you can read/copy straight from AdvantageScope.
        RobotState.packet.put("PathDiag/summary", String.format(
                "hErr=%.1f angVel=%.1f tErr=%.2f vel=%.2f busy=%.0f hold=%s",
                lastHeadingErrDeg, lastHeadingAngVelDps, transErrIn, velIps, busyMs, holdout));
        RobotState.packet.put("PathDiag/completion_log", completionLog.toString());
    }

    /** Normalizes an angle in degrees to [0, 360) so target/actual heading plots overlay cleanly. */
    private static double normalizeDeg360(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    public void stop() {
        // Wrap hardware operations in try-catch to prevent "expansion hub stopped responding"
        // errors during OpMode shutdown when the hub communication is already terminating
        try {
            if (teleOpControlEnabled) {
                follower.startTeleopDrive(true);
                follower.setTeleOpDrive(0 , 0 , 0 , robotCentric);
            } else {
                follower.breakFollowing();
            }
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown - hub may already be disconnecting
        }

        try {
            driveMotors.stop();
        } catch (Exception ignored) {
            // Ignore exceptions during shutdown
        }

        activeMode = DriveMode.NORMAL;
        lastCommandForward = 0.0;
        lastCommandStrafeLeft = 0.0;
        lastCommandTurn = 0.0;
        lastAimErrorRad = Double.NaN;
    }

    protected double readCurrentAmps(DcMotorEx motor) {
        if (motor == null) {
            return Double.NaN;
        }
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    protected double sumCurrentAmps(double... values) {
        double total = 0.0;
        for (double value : values) {
            if (Double.isFinite(value)) {
                total += value;
            }
        }
        return total;
    }

    public void driveTeleOp(double fieldX ,
                            double fieldY ,
                            double rotationInput ,
                            boolean slowMode ,
                            boolean rampMode
    ) {
        if (robotCentric != robotCentricConfig) {
            setRobotCentric(robotCentricConfig);
        }
        driveScaled(fieldX , fieldY , rotationInput , slowMode , rampMode);
    }

    /**
     * Applies scaled translation/rotation commands that have already been sampled from an input device.
     *
     * @param fieldX        translation along the field X axis (+right from the driver wall)
     * @param fieldY        translation along the field Y axis (+forward away from driver wall)
     * @param rotationInput rotation command with positive meaning counter-clockwise
     * @param slowMode      when true applies the configured slow multiplier
     */
    public void driveScaled(double fieldX , double fieldY , double rotationInput , boolean slowMode) {
        driveScaled(fieldX , fieldY , rotationInput , slowMode , false);
    }

    public void driveScaled(double fieldX , double fieldY , double rotationInput , boolean slowMode , boolean rampMode) {
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = RobotState.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestRotation = rotationInput;
        lastRequestSlowMode = slowMode;

        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double slowTurnMultiplier = Range.clip(teleOpDriveConfig.slowTurnMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double normalTurnMultiplier = Range.clip(teleOpDriveConfig.normalTurnMultiplier , 0.0 , 1.0);
        double driveMultiplier = slowMode ? slowMultiplier : normalMultiplier;
        double turnMultiplier = slowMode ? slowTurnMultiplier : normalTurnMultiplier;
        double targetForward = Range.clip(fieldY * driveMultiplier , - 1.0 , 1.0);
        double targetStrafeLeft = Range.clip(- fieldX * driveMultiplier , - 1.0 , 1.0);
        double targetTurnCW = Range.clip(- rotationInput * turnMultiplier , - 1.0 , 1.0);

        double appliedForward = targetForward;
        double appliedStrafeLeft = targetStrafeLeft;
        double appliedTurn = targetTurnCW;

        if (rampMode) {
            long now = System.nanoTime();
            if (! rampModeActive) {
                rampForward = lastCommandForward;
                rampStrafeLeft = lastCommandStrafeLeft;
                rampTurn = lastCommandTurn;
                rampModeActive = true;
                lastRampUpdateNs = now;
            }
            double dt = lastRampUpdateNs == 0L ? 0.0 : (now - lastRampUpdateNs) / 1_000_000_000.0;
            lastRampUpdateNs = now;
            if (dt <= 0.0) {
                dt = Math.max(0.0 , rampConfig.fallbackDtSeconds);
            }
            double forwardRate = Math.max(0.0 , rampConfig.forwardRatePerSec);
            double strafeRate = Math.max(0.0 , rampConfig.strafeRatePerSec);
            double turnRate = Math.max(0.0 , rampConfig.turnRatePerSec);
            double maxForwardDelta = forwardRate * dt;
            double maxStrafeDelta = strafeRate * dt;
            double maxTurnDelta = turnRate * dt;
            rampForward = moveToward(rampForward , targetForward , maxForwardDelta);
            rampStrafeLeft = moveToward(rampStrafeLeft , targetStrafeLeft , maxStrafeDelta);
            rampTurn = moveToward(rampTurn , targetTurnCW , maxTurnDelta);
            appliedForward = rampForward;
            appliedStrafeLeft = rampStrafeLeft;
            appliedTurn = rampTurn;
        } else {
            rampModeActive = false;
            rampForward = appliedForward;
            rampStrafeLeft = appliedStrafeLeft;
            rampTurn = appliedTurn;
            lastRampUpdateNs = System.nanoTime();
        }


        lastCommandForward = appliedForward;
        lastCommandStrafeLeft = appliedStrafeLeft;
        lastCommandTurn = appliedTurn;
        follower.setTeleOpDrive(appliedForward , appliedStrafeLeft , appliedTurn , robotCentric);
        activeMode = slowMode ? DriveMode.SLOW : DriveMode.NORMAL;
    }

    private static double moveToward(double current , double target , double maxDelta) {
        double delta = target - current;
        if (Math.abs(delta) <= maxDelta) {
            return target;
        }
        return current + Math.copySign(maxDelta , delta);
    }

    /**
     * Geometry-based aiming: Calculates angle from robot pose to basket centroid using atan2.
     * Uses odometry pose and basket target coordinates (tunable via FTC Dashboard).
     * Falls back to vision-based angle if available and relocalization is enabled.
     * This is an open-loop turn controller (kP + static). It does **not** use Pedro's heading PID
     * so you can see the raw turn command in telemetry when tuning.
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDrive(double fieldX , double fieldY , boolean slowMode) {
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = RobotState.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);

        // Calculate target heading using pinpoint odometry + basket target coordinates
        Pose pose = follower.getPose();
        Pose targetPose = (alliance == Alliance.RED)
            ? FieldConstants.getRedBasketTarget()
            : FieldConstants.getBlueBasketTarget();
        double targetHeading = FieldConstants.getAimAngleTo(pose , targetPose);

        // Get per-robot aim assist config
        DriveAimAssistConfig aimConfig = aimAssist;

        double headingErrorRad = normalizeAngle(targetHeading - follower.getHeading());
        double headingErrorDeg = Math.toDegrees(headingErrorRad);
        double absHeadingErrorDeg = Math.abs(headingErrorDeg);
        boolean withinDeadband = absHeadingErrorDeg <= aimConfig.deadbandDeg;
        long now = System.nanoTime();
        double dt = lastAimUpdateNs == 0L ? 0.0 : (now - lastAimUpdateNs) / 1_000_000_000.0;

        // Apply a small deadband to keep the robot still when settled; telemetry matches what is commanded
        if (withinDeadband) {
            headingErrorRad = 0.0;
            headingErrorDeg = 0.0;
        }

        // PD controller with optional inner-zone P, static feedforward, and slew limit
        double pGain = absHeadingErrorDeg <= aimConfig.innerZoneDeg ? aimConfig.kPInner : aimConfig.kP;
        double pTerm = pGain * headingErrorRad;
        double errorRateRadPerSec = (dt > 0.0 && !Double.isNaN(lastAimErrorRad)) ? (headingErrorRad - lastAimErrorRad) / dt : 0.0;
        double dTerm = aimConfig.kD * errorRateRadPerSec;
        double turnRaw = pTerm + dTerm;

        boolean staticApplied = false;
        if (headingErrorRad != 0.0
                && Math.abs(turnRaw) < aimConfig.kStatic
                && absHeadingErrorDeg > aimConfig.staticApplyAboveDeg) {
            turnRaw = Math.copySign(aimConfig.kStatic, headingErrorRad);
            staticApplied = true;
        }

        double maxTurn = Math.max(0.0, aimConfig.kMaxTurn);
        double turnClipped = Range.clip(turnRaw, -maxTurn, maxTurn);

        double turn = turnClipped;
        if (aimConfig.turnSlewRatePerSec > 0.0 && dt > 0.0) {
            double maxDelta = aimConfig.turnSlewRatePerSec * dt;
            double delta = turnClipped - lastAimTurnCommand;
            if (delta > maxDelta) {
                turn = lastAimTurnCommand + maxDelta;
            } else if (delta < -maxDelta) {
                turn = lastAimTurnCommand - maxDelta;
            }
        }

        lastAimErrorRad = headingErrorRad;
        lastAimUpdateNs = now;
        lastAimTurnCommand = turn;
        double errorRateDegPerSec = Math.toDegrees(errorRateRadPerSec);

        if (TelemetrySettings.isVerbose()) {
            RobotState.packet.put("Command/AimAndDrive/Aim Error (deg)", headingErrorDeg);
            RobotState.packet.put("Command/AimAndDrive/Aim Error Rate (deg/s)", errorRateDegPerSec);
            RobotState.packet.put("Command/AimAndDrive/Aim Target Heading (deg)", Math.toDegrees(targetHeading));
            RobotState.packet.put("Command/AimAndDrive/Aim Odo Heading (deg)", Math.toDegrees(follower.getHeading()));
            RobotState.packet.put("Command/AimAndDrive/P Gain", pGain);
            RobotState.packet.put("Command/AimAndDrive/P Term", pTerm);
            RobotState.packet.put("Command/AimAndDrive/D Term", dTerm);
            RobotState.packet.put("Command/AimAndDrive/Static Applied", staticApplied);
            RobotState.packet.put("Command/AimAndDrive/Turn Cmd Raw", turnRaw);
            RobotState.packet.put("Command/AimAndDrive/Turn Cmd (slewed)", turn);
        }

        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward, strafeLeft, turn, robotCentric);
    }

    /**
     * Fixed-angle aiming: Rotates to a fixed heading based on alliance.
     * Simple and predictable - works well from specific field positions (e.g., launch line).
     * No pose or vision calculations needed.
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDriveFixedAngle(double fieldX , double fieldY , boolean slowMode) {
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = RobotState.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);

        // Get fixed target heading based on alliance
        DriveFixedAngleAimConfig aimConfig = fixedAngleAim;
        double targetHeadingDeg = (alliance == Alliance.RED)
            ? aimConfig.redHeadingDeg
            : aimConfig.blueHeadingDeg;
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        // Calculate heading error
        double headingError = normalizeAngle(targetHeadingRad - follower.getHeading());
        lastAimErrorRad = headingError;

        // Simple P controller
        double maxTurn = Math.max(0.0, aimConfig.kMaxTurn);
        double turn = Range.clip(aimConfig.kP * headingError, -maxTurn, maxTurn);

        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward, strafeLeft, turn, robotCentric);
    }

    /**
     * Right trigger fixed-angle aiming: Rotates to a fixed heading based on alliance.
     * Uses different target angles than triangle button (265° blue / 275° red).
     * Driver retains full translation control while holding right trigger.
     *
     * @param fieldX Driver's X input (strafe)
     * @param fieldY Driver's Y input (forward)
     * @param slowMode Whether slow mode is active
     */
    public void aimAndDriveRightTriggerFixedAngle(double fieldX , double fieldY , boolean slowMode) {
        // Invert controls for Red alliance so driver perspective matches their side of field
        Alliance alliance = RobotState.getAlliance();
        if (alliance == Alliance.RED) {
            fieldX = -fieldX;
            fieldY = -fieldY;
        }

        lastRequestFieldX = fieldX;
        lastRequestFieldY = fieldY;
        lastRequestSlowMode = slowMode;
        double slowMultiplier = Range.clip(teleOpDriveConfig.slowMultiplier , 0.0 , 1.0);
        double normalMultiplier = Range.clip(teleOpDriveConfig.normalMultiplier , 0.0 , 1.0);
        double multiplier = slowMode ? slowMultiplier : normalMultiplier;
        double forward = Range.clip(fieldY * multiplier , - 1.0 , 1.0);
        double strafeLeft = Range.clip(- fieldX * multiplier , - 1.0 , 1.0);

        // Get right trigger fixed target heading based on alliance
        DriveRightTriggerFixedAngleConfig aimConfig = rightTriggerFixedAngle;
        double targetHeadingDeg = (alliance == Alliance.RED)
            ? aimConfig.redParkHeadingDeg
            : aimConfig.blueParkHeadingDeg;
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        // Calculate heading error
        double headingError = normalizeAngle(targetHeadingRad - follower.getHeading());
        lastAimErrorRad = headingError;

        // Simple P controller
        double maxTurn = Math.max(0.0, aimConfig.kMaxTurn);
        double turn = Range.clip(aimConfig.kP * headingError, -maxTurn, maxTurn);

        lastCommandForward = forward;
        lastCommandStrafeLeft = strafeLeft;
        lastCommandTurn = turn;
        lastRequestRotation = turn;

        follower.setTeleOpDrive(forward, strafeLeft, turn, robotCentric);
    }

    public double getLastAimErrorRadians() {
        return lastAimErrorRad;
    }

    /**
     * Returns true if the robot is effectively aimed at the goal based on the active alliance and pose.
     * Uses odometry pose and a tight heading deadband.
     */
    public boolean isAimSettled(double headingDeadbandDeg) {
        Pose pose = follower.getPose();
        if (pose == null) {
            return false;
        }

        // Require low chassis speed
        if (getRobotSpeedInchesPerSecond() > STATIONARY_SPEED_THRESHOLD_IN_PER_SEC) {
            return false;
        }

        Alliance alliance = RobotState.getAlliance();
        Pose targetPose = (alliance == Alliance.RED)
            ? FieldConstants.getRedBasketTarget()
            : FieldConstants.getBlueBasketTarget();
        double targetHeading = FieldConstants.getAimAngleTo(pose , targetPose);
        double headingErrorRad = normalizeAngle(targetHeading - follower.getHeading());
        double headingErrorDeg = Math.toDegrees(Math.abs(headingErrorRad));
        return headingErrorDeg <= headingDeadbandDeg;
    }

    public boolean isRobotStationary() {
        return getRobotSpeedInchesPerSecond() <= STATIONARY_SPEED_THRESHOLD_IN_PER_SEC;
    }

    // --- Telemetry accessors ------------------------------------------------
    public DriveMode getDriveMode() {
        return activeMode;
    }

    public Pose2D getPose() {
        Pose pose = follower.getPose();
        if (pose == null) {
            return new Pose2D(DistanceUnit.INCH , 0.0 , 0.0 , AngleUnit.RADIANS , 0.0);
        }
        return new Pose2D(
                DistanceUnit.INCH ,
                pose.getX() ,
                pose.getY() ,
                AngleUnit.RADIANS ,
                follower.getHeading()
        );
    }

    public Pose getFollowerPose() {
        return follower.getPose();
    }

    public Follower getFollower() {
        return follower;
    }

    public void updateFollower() {
        follower.update();
    }

    public void followPath(PathChain pathChain , boolean holdPositionAtEnd) {
        follower.followPath(pathChain , holdPositionAtEnd);
    }

    public void followPath(PathChain pathChain , double maxPower , boolean holdPositionAtEnd) {
        double clippedPower = Range.clip(maxPower , 0.0 , 1.0);
        follower.followPath(pathChain , clippedPower , holdPositionAtEnd);
    }

    public boolean isFollowerBusy() {
        return follower.isBusy();
    }

    public double getLfPower() {
        return motorLf.getPower();
    }

    public double getRfPower() {
        return motorRf.getPower();
    }

    public double getLbPower() {
        return motorLb.getPower();
    }

    public double getRbPower() {
        return motorRb.getPower();
    }

    public double getLastCommandDrive() {
        return lastCommandForward;
    }

    public double getLastCommandStrafe() {
        return lastCommandStrafeLeft;
    }

    public double getLastCommandTurn() {
        return lastCommandTurn;
    }

    public double getLfVelocityTicksPerSec() {
        return motorLf.getVelocity();
    }

    public double getRfVelocityTicksPerSec() {
        return motorRf.getVelocity();
    }

    public double getLbVelocityTicksPerSec() {
        return motorLb.getVelocity();
    }

    public double getRbVelocityTicksPerSec() {
        return motorRb.getVelocity();
    }

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle) , Math.cos(angle));
    }

    private boolean isGoalAprilTag(int tagId) {
        return tagId == FieldConstants.BLUE_GOAL_TAG_ID || tagId == FieldConstants.RED_GOAL_TAG_ID;
    }

    private boolean isVisionPoseFresh() {
        long ageMs = System.currentTimeMillis() - vision.getLastPoseTimestampMs();
        return ageMs <= MAX_VISION_AGE_MS;
    }

    private double mt2DistanceThresholdInches() { return visionRelocalizeConfig.MT2DistanceThreshold; }

    private double headingErrorDegrees(Pose mt1 , Pose mt2) {
        double headingErrorRad = normalizeAngle(mt1.getHeading() - mt2.getHeading());
        return Math.abs(Math.toDegrees(headingErrorRad));
    }

    private boolean posesAgreeForMt2(Pose mt1 , Pose mt2) {
        double distanceInches = distanceBetween(mt1 , mt2);
        double headingErrorDeg = headingErrorDegrees(mt1 , mt2);

        return distanceInches < mt2DistanceThresholdInches()
                && headingErrorDeg < visionRelocalizeConfig.MT2HeadingThresholdDeg;
    }

    private void recordVisionAgreementTelemetry(double distance , double headingErrorDeg , boolean mt2Safe) {
        if (TelemetrySettings.isVerbose()) {
            RobotState.packet.put("/vision/Poses/distanceBetweenMT1MT2", distance);
            RobotState.packet.put("/vision/Poses/relocalizeWithMT2", mt2Safe);
            RobotState.packet.put("/vision/Poses/headingErrorMT1MT2", headingErrorDeg);
            RobotState.packet.put("/vision/Poses/mt2DistanceThreshold", mt2DistanceThresholdInches());
            RobotState.packet.put("/vision/Poses/mt2HeadingThresholdDeg", visionRelocalizeConfig.MT2HeadingThresholdDeg);
        }
    }

    private void recordVisionPoseTelemetry(Pose mt1 , Pose mt2) {
        if (TelemetrySettings.isVerbose()) {
            double mt1HeadingDeg = mt1 == null ? Double.NaN : Math.toDegrees(mt1.getHeading());
            double mt2HeadingDeg = mt2 == null ? Double.NaN : Math.toDegrees(mt2.getHeading());

            RobotState.packet.put("/vision/Poses/mt1/x", mt1 == null ? Double.NaN : mt1.getX());
            RobotState.packet.put("/vision/Poses/mt1/y", mt1 == null ? Double.NaN : mt1.getY());
            RobotState.packet.put("/vision/Poses/mt1/heading_deg", mt1HeadingDeg);

            RobotState.packet.put("/vision/Poses/mt2/x", mt2 == null ? Double.NaN : mt2.getX());
            RobotState.packet.put("/vision/Poses/mt2/y", mt2 == null ? Double.NaN : mt2.getY());
            RobotState.packet.put("/vision/Poses/mt2/heading_deg", mt2HeadingDeg);
        }
    }

    private void recordRelocalizeSource(String source , boolean success , long timestampMs , int tagId) {
        lastRelocalizeSource = source;
        if (TelemetrySettings.isVerbose()) {
            RobotState.packet.put("/vision/relocalize/last/source", source);
            RobotState.packet.put("/vision/relocalize/last/success", success);
            RobotState.packet.put("/vision/relocalize/last/timestamp_ms", timestampMs);
            RobotState.packet.put("/vision/relocalize/last/tag", tagId);
        }
    }

    private void recordAutoRelocalizePacket(boolean success , long timestampMs , int tagId , String status , String source) {
        if (TelemetrySettings.isVerbose()) {
            RobotState.packet.put("/vision/relocalize/auto/success", success);
            RobotState.packet.put("/vision/relocalize/auto/timestamp_ms", timestampMs);
            RobotState.packet.put("/vision/relocalize/auto/tag", tagId);
            RobotState.packet.put("/vision/relocalize/auto/status", status);
            RobotState.packet.put("/vision/relocalize/auto/source", source);
        }
    }

    private boolean poseWithinField(Pose pose) {
        if (pose == null) {
            return false;
        }
        double x = pose.getX();
        double y = pose.getY();
        double min = -6.0; // allow small margin
        double max = FieldConstants.FIELD_WIDTH_INCHES + 6.0;
        return x >= min && x <= max && y >= min && y <= max;
    }

    private boolean headingFacesTag(Pose pose, int tagId, double maxHeadingErrorDeg) {
        if (pose == null) {
            return false;
        }
        Pose tagPosePedro;
        if (tagId == FieldConstants.BLUE_GOAL_TAG_ID) {
            tagPosePedro = FieldConstants.BLUE_GOAL_TAG_APRILTAG; // Pedro frame already
        } else if (tagId == FieldConstants.RED_GOAL_TAG_ID) {
            tagPosePedro = FieldConstants.RED_GOAL_TAG_APRILTAG; // Pedro frame already
        } else {
            return false;
        }
        double bearing = Math.atan2(tagPosePedro.getY() - pose.getY(), tagPosePedro.getX() - pose.getX());
        double headingErr = normalizeAngle(pose.getHeading() - bearing);
        double headingErrDeg = Math.toDegrees(Math.abs(headingErr));
        return headingErrDeg <= maxHeadingErrorDeg;
    }

    private void flashRelocalizeFeedback() {
        if (lighting != null) {
            lighting.flashAimAligned();
        }
    }

    private void maybeRelocalizeFromVision() {
        if (! visionRelocalizationEnabled) {
            return;
        }
        if (! vision.shouldUpdateOdometry()) {
            return;
        }

        double speed = getRobotSpeedInchesPerSecond();
        if (speed > STATIONARY_SPEED_THRESHOLD_IN_PER_SEC) {
            return;
        }

        // Use current odometry as reference - reject large jumps during automatic relocalization
        Pose currentOdometry = follower.getPose();
        if (forceRelocalizeFromVision(currentOdometry)) {
            // Successful re-localization already recorded inside the helper.
        }
    }

    /**
     * Attempts a single relocalization tied to a launch event.
     * Gated to moments where the robot should be steady and facing the goal tag.
     */
    public void tryRelocalizeForShot() {
        if (!visionRelocalizationEnabled) {
            recordAutoRelocalizePacket(false , System.currentTimeMillis() , -1 , "launch relocalize failed" , "disabled");
            return;
        }
        if (vision == null || !vision.hasValidTag()) {
            recordAutoRelocalizePacket(false , System.currentTimeMillis() , -1 , "launch relocalize failed" , "no_tag");
            return;
        }

        long nowMs = System.currentTimeMillis();
        if (nowMs - lastAutoRelocalizeMs < 300) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "cooldown");
            return; // simple cooldown
        }

        // Require low chassis speed and minimal commanded turn to avoid relocalizing while spinning
        if (getRobotSpeedInchesPerSecond() > STATIONARY_SPEED_THRESHOLD_IN_PER_SEC) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "too_fast");
            return;
        }
        if (Math.abs(lastCommandTurn) > 0.05) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "turning");
            return;
        }

        long ageMs = nowMs - vision.getLastPoseTimestampMs();
        if (ageMs > MAX_VISION_AGE_MS) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "stale_pose");
            return;
        }

        VisionSubsystemLimelight.TagSnapshot snapshot = vision.getLastSnapshot();
        if (snapshot == null) {
            recordAutoRelocalizePacket(false , nowMs , -1 , "launch relocalize failed" , "no_snapshot");
            return;
        }

        if (!isGoalAprilTag(snapshot.tagId)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "not_goal_tag");
            return;
        }

        Pose pedroPoseMT1 = snapshot.pedroPoseMT1;
        Pose pedroPoseMT2 = snapshot.pedroPoseMT2;
        if (pedroPoseMT1 == null && pedroPoseMT2 == null) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "no_pose");
            return;
        }

        // Geometry-based sanity checks (field bounds, facing tag, distance)
        Pose primaryPose = pedroPoseMT2 != null ? pedroPoseMT2 : pedroPoseMT1;
        if (!poseWithinField(primaryPose)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "off_field");
            return;
        }
        if (!headingFacesTag(primaryPose, snapshot.tagId, 120.0)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "heading_not_facing_tag");
            return;
        }
        double distToTag = Double.NaN;
        Pose tagPosePedro = snapshot.tagId == FieldConstants.BLUE_GOAL_TAG_ID
                ? FieldConstants.BLUE_GOAL_TAG_APRILTAG
                : FieldConstants.RED_GOAL_TAG_APRILTAG;
        if (tagPosePedro != null && primaryPose != null) {
            distToTag = distanceBetween(primaryPose, tagPosePedro);
        }
        if (!Double.isNaN(distToTag) && (distToTag < 6.0 || distToTag > 200.0)) {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "distance_out_of_range");
            return;
        }

        boolean mt2Safe = pedroPoseMT1 != null && pedroPoseMT2 != null && posesAgreeForMt2(pedroPoseMT1, pedroPoseMT2);
        if (!mt2Safe && pedroPoseMT1 != null && pedroPoseMT2 != null) {
            recordVisionAgreementTelemetry(distanceBetween(pedroPoseMT1, pedroPoseMT2), headingErrorDegrees(pedroPoseMT1, pedroPoseMT2), mt2Safe);
        }

        // Use current odometry as reference - reject large jumps during shot correction
        Pose currentOdometry = follower.getPose();
        if (forceRelocalizeFromVision(currentOdometry)) {
            lastAutoRelocalizeMs = nowMs;
            visionRelocalizeStatus = "Re-localized on launch";
            visionRelocalizeStatusMs = nowMs;
            recordAutoRelocalizePacket(true , nowMs , snapshot.tagId , visionRelocalizeStatus , lastRelocalizeSource);
//            flashRelocalizeFeedback();
        }
        else {
            recordAutoRelocalizePacket(false , nowMs , snapshot.tagId , "launch relocalize failed" , "blocked_conditions");
        }
    }

    /**
     * Forces the follower pose to match the latest Limelight pose when available.
     *
     * @return {@code true} when the pose was updated.
     */
    public void tryRelocalize() {
        boolean tagVisible = vision.hasValidTag();

        if (tagVisible &&
                (vision.getCurrentTagId() == FieldConstants.BLUE_GOAL_TAG_ID ||
                        vision.getCurrentTagId() == FieldConstants.RED_GOAL_TAG_ID)) {
            boolean success = forceRelocalizeFromVision();
            if (success) {
                visionRelocalizeStatus = "Pose updated from Limelight";
            } else {
                visionRelocalizeStatus = "Failed to update pose from Limelight";
            }
        } else if (tagVisible) {
            visionRelocalizeStatus = "No Goal AprilTag visible";
        } else {
            visionRelocalizeStatus = "No AprilTag visible";
        }
        visionRelocalizeStatusMs = System.currentTimeMillis();
    }

    /**
     * Re-centers odometry heading to the configured field-forward angle.
     * Driver should square the robot to the field wall (start-heading direction)
     * before triggering this for a quick recovery after a bad auto handoff.
     */
    public void resetHeadingToFieldForward() {
        Pose currentPose = follower.getPose();
        if (currentPose == null) {
            return;
        }

        double targetHeading = Math.toRadians(initialPoseConfig.startHeadingDeg);
        Pose correctedPose = new Pose(currentPose.getX() , currentPose.getY() , targetHeading);

        follower.setPose(correctedPose);
        vision.markOdometryUpdated();

        visionRelocalizeStatus = "Heading reset to field-forward (square to wall)";
        visionRelocalizeStatusMs = System.currentTimeMillis();
    }
    /**
     * Applies a relocalized pose by resetting the FusionLocalizer's Kalman
     * covariance AND calling Pedro's follower.setPose. Both calls are needed:
     * the localizer update flushes filter uncertainty so the next vision
     * correction doesn't pull the pose around against the manual override;
     * the follower update keeps Pedro's internal path-tracking T-value
     * consistent with the new pose.
     */
    private void applyRelocalizedPose(Pose pose) {
        if (Constants.activeFusionLocalizer != null) {
            Constants.activeFusionLocalizer.forceRelocalize(
                    pose,
                    DriveFusionConfig.relocalizeCovarianceXY,
                    Math.toRadians(DriveFusionConfig.relocalizeCovarianceHeadingDeg));
        }
        follower.setPose(pose);
    }

    /**
     * Forces relocalization from vision WITHOUT jump safeguards.
     * Use ONLY for manual driver-initiated relocalization (X button) where the driver
     * intentionally wants to reset pose after a crash or bad state.
     *
     * For all other relocalization (auto init, shot correction), use the overloaded
     * version with a reference pose for jump safeguards.
     *
     * @return {@code true} when the pose was updated.
     */
    public boolean forceRelocalizeFromVision() {
        // No reference pose = no jump safeguards (manual recovery scenario)
        return forceRelocalizeFromVisionInternal(null, false);
    }

    /**
     * Forces relocalization from vision WITH jump safeguards.
     * Compares the proposed vision pose against the reference pose and rejects
     * if the jump exceeds configured thresholds.
     *
     * @param referencePose The pose to compare against (expected start pose or current odometry).
     *                      If null, no jump check is performed (same as forceRelocalizeFromVision()).
     * @return {@code true} when the pose was updated.
     */
    public boolean forceRelocalizeFromVision(Pose referencePose) {
        return forceRelocalizeFromVisionInternal(referencePose, false);
    }

    /**
     * Forces relocalization from vision against a TRUSTED reference pose.
     * Unlike the untrusted overload, the jump check is applied even in the
     * HEADING_UNKNOWN seed branch. Use for auto init, where the robot is placed
     * at a known start pose, so a far-off single-tag solution should be rejected.
     *
     * @param referencePose The trusted reference (e.g. the configured auto start pose).
     * @return {@code true} when the pose was updated.
     */
    public boolean forceRelocalizeFromVisionTrusted(Pose referencePose) {
        return forceRelocalizeFromVisionInternal(referencePose, true);
    }

    /**
     * Internal implementation of vision relocalization with optional jump safeguards.
     *
     * @param referencePose    The pose to compare against for jump limits. Null to skip jump check.
     * @param trustedReference When true, the jump check also gates the HEADING_UNKNOWN
     *                         seed branch (the reference is known-good, e.g. auto start).
     * @return {@code true} when the pose was updated.
     */
    private boolean forceRelocalizeFromVisionInternal(Pose referencePose, boolean trustedReference) {
        long nowMs = System.currentTimeMillis();
        if (!DriveFusionConfig.fusionVisionEnabled) {
            recordRelocalizeSource("fusion_disabled", false, nowMs, -1);
            return false;
        }
        VisionSubsystemLimelight.TagSnapshot snap = vision.getLastSnapshot();
        if (snap == null) {
            recordRelocalizeSource("none", false, nowMs, -1);
            return false;
        }


        // 1. Use MT1 whenever heading is unknown
        Pose pedroPoseMT1 = snap.pedroPoseMT1;
        Pose pedroPoseMT2 = snap.pedroPoseMT2;

        if (visionState == HEADING_UNKNOWN) {
            if (pedroPoseMT1 == null) {
                recordRelocalizeSource("mt1_missing", false, nowMs, snap.tagId);
                return false;
            }

            // Untrusted reference (e.g. teleop's startHeadingDeg config default):
            // skip the jump check, since rejecting "big jumps" against an unknown
            // current pose would block exactly the corrections we need when the
            // robot is placed at a different heading than the config. With a TRUSTED
            // reference (auto start pose), still gate it — a far single-tag solution
            // must not hijack a known start placement.
            if (trustedReference && referencePose != null
                    && !isPoseJumpAllowed(pedroPoseMT1, referencePose, "mt1_seed")) {
                recordRelocalizeSource("mt1_seed_rejected", false, nowMs, snap.tagId);
                return false;
            }
            applyRelocalizedPose(pedroPoseMT1);
            vision.markOdometryUpdated();

            // Now heading is calibrated — clear any stale disagreement count
            visionState = HEADING_KNOWN;
            consecutiveMt2DisagreementCount = 0;
            recordRelocalizeSource("mt1_seed", true, nowMs, snap.tagId);
            return true;
        }

        // 2. If heading is known, prefer MT2 when it agrees with MT1 (or MT1 missing)
        boolean mt2Safe = pedroPoseMT1 != null && pedroPoseMT2 != null && posesAgreeForMt2(pedroPoseMT1, pedroPoseMT2);

        // Detect wrong heading seed: if both poses are visible but consistently disagree,
        // the initial MT1 seed likely picked the wrong ambiguity solution (180° flip).
        // Reset to HEADING_UNKNOWN so step 1 re-seeds on the next call.
        if (pedroPoseMT1 != null && pedroPoseMT2 != null) {
            if (mt2Safe) {
                consecutiveMt2DisagreementCount = 0;
            } else {
                consecutiveMt2DisagreementCount++;
                RobotState.packet.put("/vision/mt2DisagreementCount", consecutiveMt2DisagreementCount);
                if (consecutiveMt2DisagreementCount >= visionRelocalizeConfig.mt2DisagreementResetFrames) {
                    visionState = HEADING_UNKNOWN;
                    consecutiveMt2DisagreementCount = 0;
                    recordRelocalizeSource("mt2_heading_reset", false, nowMs, snap.tagId);
                    return false;
                }
            }
        }

        if (pedroPoseMT2 != null && (mt2Safe || pedroPoseMT1 == null)) {
            // Check jump limits against reference pose
            if (referencePose != null && !isPoseJumpAllowed(pedroPoseMT2, referencePose, "mt2")) {
                return false;
            }

            applyRelocalizedPose(pedroPoseMT2);
            vision.markOdometryUpdated();
            recordRelocalizeSource("mt2", true, nowMs, snap.tagId);
            return true;
        }

        // 3. Fallback to MT1 if MT2 unavailable or unsafe
        if (pedroPoseMT1 != null) {
            // Check jump limits against reference pose
            if (referencePose != null && !isPoseJumpAllowed(pedroPoseMT1, referencePose, "mt1")) {
                return false;
            }

            applyRelocalizedPose(pedroPoseMT1);
            vision.markOdometryUpdated();
            recordRelocalizeSource("mt1", true, nowMs, snap.tagId);
            return true;
        }

        recordRelocalizeSource("none", false, nowMs, snap.tagId);
        return false;
    }

    /**
     * Checks if a proposed vision pose is within allowed jump limits of the reference pose.
     * Logs rejection telemetry if the jump is too large.
     *
     * @param proposedPose The vision-reported pose
     * @param referencePose The pose to compare against (expected start or current odometry)
     * @param source Label for telemetry logging (e.g., "mt1", "mt2", "mt1_seed")
     * @return true if the jump is allowed, false if it exceeds limits
     */
    private boolean isPoseJumpAllowed(Pose proposedPose, Pose referencePose, String source) {
        if (proposedPose == null || referencePose == null) {
            return true; // Can't check, allow by default
        }

        double jumpDistance = distanceBetween(proposedPose, referencePose);
        double jumpHeadingDeg = Math.abs(Math.toDegrees(normalizeAngle(
                proposedPose.getHeading() - referencePose.getHeading())));

        double maxDistance = visionRelocalizeConfig.maxJumpDistanceInches;
        double maxHeadingDeg = visionRelocalizeConfig.maxJumpHeadingDeg;

        boolean distanceOk = jumpDistance <= maxDistance;
        boolean headingOk = jumpHeadingDeg <= maxHeadingDeg;

        // Log jump diagnostics
        RobotState.packet.put("/vision/relocalize/jump/distance_in", jumpDistance);
        RobotState.packet.put("/vision/relocalize/jump/heading_deg", jumpHeadingDeg);
        RobotState.packet.put("/vision/relocalize/jump/max_distance_in", maxDistance);
        RobotState.packet.put("/vision/relocalize/jump/max_heading_deg", maxHeadingDeg);
        RobotState.packet.put("/vision/relocalize/jump/distance_ok", distanceOk);
        RobotState.packet.put("/vision/relocalize/jump/heading_ok", headingOk);

        if (!distanceOk || !headingOk) {
            String reason = !distanceOk && !headingOk ? "distance_and_heading"
                    : !distanceOk ? "distance" : "heading";
            recordRelocalizeSource(source + "_jump_rejected_" + reason, false,
                    System.currentTimeMillis(), -1);
            RobotState.packet.put("/vision/relocalize/jump/rejected", true);
            RobotState.packet.put("/vision/relocalize/jump/reject_reason", reason);
            return false;
        }

        RobotState.packet.put("/vision/relocalize/jump/rejected", false);
        return true;
    }

    private void maybeFeedVisionMeasurement() {
        if (!DriveFusionConfig.fusionVisionEnabled) return;
        if (fusionLocalizer == null || vision == null) return;
        VisionSubsystemLimelight.TagSnapshot snap = vision.getLastSnapshot();
        if (snap == null) return;
        // Reject stale snapshots (e.g. one captured during init while repositioning)
        // so they can't be replayed into the filter on the first auto loop.
        long ageMs = (System.nanoTime() - snap.capturedAtNs) / 1_000_000L;
        if (ageMs > DriveFusionConfig.maxMeasurementAgeMs) return;
        if (snap.capturedAtNs == lastFedSnapshotNs) return;
        lastFedSnapshotNs = snap.capturedAtNs;
        if (snap.decisionMargin < DriveFusionConfig.minTargetAreaPercent) return;
        // MT2 is heading-dependent and can be badly wrong when the heading seed is
        // off (observed off-field during init while MT1 read fine). Only trust MT2
        // when it agrees with MT1 — same gate the relocalize path uses — otherwise
        // fall back to the heading-independent MT1. The filter only consumes X/Y
        // from the measurement anyway (heading always comes from Pinpoint).
        Pose mt1 = snap.pedroPoseMT1;
        Pose mt2 = snap.pedroPoseMT2;
        boolean mt2Safe = mt1 != null && mt2 != null && posesAgreeForMt2(mt1, mt2);
        Pose pose = (mt2 != null && (mt2Safe || mt1 == null)) ? mt2 : mt1;
        RobotState.packet.put("/vision/fusion/source", pose == mt2 ? "MT2" : "MT1");
        if (pose == null) return;
        // Sanity-gate the measurement before it enters the filter. The continuous
        // feed has no jump check of its own, so an off-field or wildly-disagreeing
        // single-tag solution would otherwise drag the fused pose off the map.
        if (!poseWithinField(pose)) {
            RobotState.packet.put("/vision/fusion/rejected", "off_field");
            return;
        }
        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            double jumpIn = Math.hypot(pose.getX() - currentPose.getX(),
                                       pose.getY() - currentPose.getY());
            RobotState.packet.put("/vision/fusion/innovation_in", jumpIn);
            if (jumpIn > DriveFusionConfig.maxFusionInnovationIn) {
                RobotState.packet.put("/vision/fusion/rejected", "jump");
                return;
            }
        }
        RobotState.packet.put("/vision/fusion/rejected", "none");
        long measurementNs = snap.capturedAtNs - (long)(DriveFusionConfig.limelightLatencyMs * 1_000_000L);
        fusionLocalizer.addMeasurement(pose, measurementNs);
    }

    public double getRobotSpeedInchesPerSecond() {
        try {
            Vector velocity = follower.getVelocity();
            if (velocity == null) {
                return Double.POSITIVE_INFINITY;
            }
            return Math.abs(velocity.getMagnitude());
        } catch (Exception ignored) {
            // Default to +Infinity so "is stationary?" callers fail closed (won't fire by accident).
            return Double.POSITIVE_INFINITY;
        }
    }

    public double getRawPinpointHeadingDeg() {
        try {
            if (follower.getPoseTracker() != null && follower.getPoseTracker().getLocalizer() != null) {
                Pose rawPose = follower.getPoseTracker().getLocalizer().getPose();
                if (rawPose != null) {
                    return Math.toDegrees(rawPose.getHeading());
                }
            }
        } catch (Exception ignored) {
            // Some localizers may not expose raw heading
        }
        return Double.NaN;
    }

    private static double distanceBetween(Pose a , Pose b) {
        return Math.hypot(a.getX() - b.getX() , a.getY() - b.getY());
    }

    protected double ticksToInchesPerSecond(double ticksPerSecond) {
        double mps = Constants.Speed.ticksPerSecToMps(ticksPerSecond);
        return DistanceUnit.METER.toInches(mps);
    }

    protected double followerVelocityIps() {
        try {
            Vector velocity = follower.getVelocity();
            if (velocity == null) {
                return 0.0;
            }
            return velocity.getMagnitude();
        } catch (Exception ignored) {
            // Telemetry-only path; falling back to 0 is preferable to crashing the loop.
            return 0.0;
        }
    }

    // ========================================================================
    // AutoLog Output Methods
    // These methods are automatically logged by KoalaLog to WPILOG files
    // and published to FTC Dashboard for AdvantageScope Lite
    // ========================================================================

    public double getPoseXInches() {
        Pose pose = follower.getPose();
        return pose != null ? pose.getX() : 0.0;
    }

    public double getPoseYInches() {
        Pose pose = follower.getPose();
        return pose != null ? pose.getY() : 0.0;
    }

    public double getPoseHeadingDeg() {
        return Math.toDegrees(follower.getHeading());
    }

    public boolean isRobotCentric() {
        return robotCentric;
    }

    public String getDriveModeString() {
        return activeMode.name();
    }

    public double getRequestFieldX() {
        return lastRequestFieldX;
    }

    public double getRequestFieldY() {
        return lastRequestFieldY;
    }

    public double getRequestRotation() {
        return lastRequestRotation;
    }

    public boolean getRequestSlowMode() {
        return lastRequestSlowMode;
    }

    public double getCommandForward() {
        return lastCommandForward;
    }

    public double getCommandStrafeLeft() {
        return lastCommandStrafeLeft;
    }

    public double getCommandTurn() {
        return lastCommandTurn;
    }

    public boolean isFollowerBusyLogged() {
        return follower.isBusy();
    }

    public double getLfPowerLogged() {
        return motorLf.getPower();
    }

    public double getRfPowerLogged() {
        return motorRf.getPower();
    }

    public double getLbPowerLogged() {
        return motorLb.getPower();
    }

    public double getRbPowerLogged() {
        return motorRb.getPower();
    }

    public double getLfCurrentAmps() {
        return readCurrentAmps(motorLf);
    }

    public double getRfCurrentAmps() {
        return readCurrentAmps(motorRf);
    }

    public double getLbCurrentAmps() {
        return readCurrentAmps(motorLb);
    }

    public double getRbCurrentAmps() {
        return readCurrentAmps(motorRb);
    }

    public double getDriveTotalCurrentAmps() {
        return sumCurrentAmps(
                getLfCurrentAmps(),
                getRfCurrentAmps(),
                getLbCurrentAmps(),
                getRbCurrentAmps()
        );
    }

    public double getLfVelocityIps() {
        return ticksToInchesPerSecond(motorLf.getVelocity());
    }

    public double getRfVelocityIps() {
        return ticksToInchesPerSecond(motorRf.getVelocity());
    }

    public double getLbVelocityIps() {
        return ticksToInchesPerSecond(motorLb.getVelocity());
    }

    public double getRbVelocityIps() {
        return ticksToInchesPerSecond(motorRb.getVelocity());
    }

    public double getFollowerSpeedIps() {
        return followerVelocityIps();
    }

    // =========================================================================
    // Ivy Command factories. Traffic Cones idiom: subsystem owns its commands.
    // Aim variants suspend the default drive command via priority + SUSPEND;
    // when the aim command ends, default resumes.
    // =========================================================================

    /** Joystick magnitudes below this are zeroed inside the drive command (joysticks never quite center). */
    private static final double DRIVE_TRANSLATION_DEADBAND = 0.05;
    private static double applyDriveDeadband(double value) {
        return Math.abs(value) < DRIVE_TRANSLATION_DEADBAND ? 0.0 : value;
    }

    /**
     * Priority-0 infinite default drive Command. Reads the driver inputs each
     * tick and passes them to driveTeleOp. Use interruptedBehavior=SUSPEND so
     * higher-priority aim commands preempt and this resumes when they end.
     */
    public com.pedropathing.ivy.Command defaultDrive(
            java.util.function.DoubleSupplier fieldX,
            java.util.function.DoubleSupplier fieldY,
            java.util.function.DoubleSupplier rotation,
            java.util.function.BooleanSupplier slow,
            java.util.function.BooleanSupplier ramp) {
        return Commands.infinite(() -> driveTeleOp(
                        fieldX.getAsDouble(),
                        fieldY.getAsDouble(),
                        rotation.getAsDouble(),
                        slow.getAsBoolean(),
                        ramp.getAsBoolean()))
                .requiring(this)
                .setPriority(0)
                .setInterruptedBehavior(com.pedropathing.ivy.behaviors.InterruptedBehavior.SUSPEND);
    }

    /**
     * Priority-1 infinite geometry aim Command. Higher priority preempts the
     * default drive; the default suspends and resumes when this is cancelled.
     */
    public com.pedropathing.ivy.Command aimAndDriveCmd(
            java.util.function.DoubleSupplier fieldX,
            java.util.function.DoubleSupplier fieldY,
            java.util.function.BooleanSupplier slow) {
        return Commands.infinite(() -> aimAndDrive(
                        applyDriveDeadband(fieldX.getAsDouble()),
                        applyDriveDeadband(fieldY.getAsDouble()),
                        slow.getAsBoolean()))
                .requiring(this)
                .setPriority(1);
    }

    /**
     * Priority-1 infinite fixed-angle aim Command (driver triangle).
     */
    public com.pedropathing.ivy.Command aimAndDriveFixedAngleCmd(
            java.util.function.DoubleSupplier fieldX,
            java.util.function.DoubleSupplier fieldY,
            java.util.function.BooleanSupplier slow) {
        return Commands.infinite(() -> aimAndDriveFixedAngle(
                        applyDriveDeadband(fieldX.getAsDouble()),
                        applyDriveDeadband(fieldY.getAsDouble()),
                        slow.getAsBoolean()))
                .requiring(this)
                .setPriority(1);
    }

    /**
     * Priority-1 infinite right-trigger fixed-angle aim Command.
     */
    public com.pedropathing.ivy.Command aimAndDriveRightTriggerCmd(
            java.util.function.DoubleSupplier fieldX,
            java.util.function.DoubleSupplier fieldY,
            java.util.function.BooleanSupplier slow) {
        return Commands.infinite(() -> aimAndDriveRightTriggerFixedAngle(
                        applyDriveDeadband(fieldX.getAsDouble()),
                        applyDriveDeadband(fieldY.getAsDouble()),
                        slow.getAsBoolean()))
                .requiring(this)
                .setPriority(1);
    }

    /**
     * One-shot Command that triggers vision-driven relocalization.
     */
    public com.pedropathing.ivy.Command tryRelocalizeCmd() {
        return Commands.instant(this::tryRelocalize).requiring(this);
    }

    /**
     * One-shot Command that resets the follower's heading to field-forward.
     */
    public com.pedropathing.ivy.Command resetHeadingCmd() {
        return Commands.instant(this::resetHeadingToFieldForward).requiring(this);
    }

}

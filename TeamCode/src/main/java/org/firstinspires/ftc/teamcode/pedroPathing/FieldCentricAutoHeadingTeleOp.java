package org.firstinspires.ftc.teamcode.pedroPathing;

import static dev.nextftc.bindings.Bindings.button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleSupplier;

// NextFTC bindings
import dev.nextftc.bindings.BindingManager;

/**
 * Field-centric mecanum TeleOp with auto-heading toward the left-stick direction.
 * 0 deg = away from the alliance wall. Right stick overrides rotation.
 * Precision Mode (LB held) suppresses auto-heading, no speed scaling.
 * Slow Mode (RB held) suppresses auto-heading, with speed scaling.
 * SDK: 11.0.0, PedroPathing: 2.0.2
 */
@Configurable
@TeleOp(name = "FieldCentricAutoHeadingTeleOp", group = "Pedro")
public class FieldCentricAutoHeadingTeleOp extends OpMode {

    private FtcDashboard dash;
    public static boolean POSE_IS_METERS = false; // set true only if your Pose units are meters

    // ===== Pedro follower and pose =====
    private Follower follower;
    public static Pose startingPose; // optional: set from Auto

    // ===== Telemetry =====
    private TelemetryManager telemetryM;

    // ===== Configurables =====
    public static double MOVE_DEADBAND = 0.08;                 // left stick
    public static double ROT_DEADBAND  = 0.10;                 // right stick
    public static double OMEGA_MAX_RAD = Math.toRadians(90);   // max yaw rate
    public static double L_CONV_M      = 2.5;                  // meters for heading convergence while moving
    public static double SLEW_RATE_UNITS_PER_S = 3.0;          // left-stick rate limit 0..1/s
    public static double DIR_RESET_DEG = 11;                   // reset path distance if desired heading jumps

    // Slow Mode scale while right bumper is held
    public static double PRECISION_MULTIPLIER = 0.40;          // scales both translation and rotation (RB only)

    // Manual override and auto-heading rearm
    public static double AUTOHEADING_REARM_TIME_S = 0.35;
    public static double MANUAL_OVERRIDE_THRESH   = 0.02;

    // Rest policy
    public static double REST_ARM_TIME_S      = 0.20;          // time after entering rest before drift snap allowed
    public static double DRIFT_TAKEOVER_DEG   = 12.0;          // snap reference to current if drift exceeds this

    // Final flag passed to setTeleOpDrive(..., field_or_robot_flag)
    // Set to the value your Pedro version expects for FIELD-CENTRIC.
    public static boolean FIELD_OR_ROBOT_FLAG = false;

    // Axis and rotation sign flips
    public static boolean INVERT_FIELD_X = false;              // invert field X (forward)
    public static boolean INVERT_FIELD_Y = true;               // invert field Y (left)
    public static boolean INVERT_ROT_STICK = false;            // flip right stick rotation if needed
    public static boolean NEGATE_FIELD_YAW = false;            // negate psi if IMU heading sign is opposite

    // ===== Zero snap so BRAKE can engage =====
    public static double ZERO_SNAP = 0.02;                     // commands with magnitude < ZERO_SNAP go to exactly 0

    // ===== Active braking (translation + rotation) =====
    public static boolean ENABLE_ACTIVE_BRAKE   = true;
    public static double  VEL_DEADBAND_MPS      = 0.02;        // ignore tiny drift
    public static double  K_BRAKE                = 0.60;        // gain from velocity to stick units
    public static double  MAX_BRAKE_STICK        = 0.35;        // cap per axis

    public static boolean ENABLE_ANGULAR_BRAKE  = true;
    public static double  OMEGA_DEADBAND_RADPS  = Math.toRadians(2.0);
    public static double  K_OMEGA_BRAKE          = 0.60;        // maps rad/s to fraction of OMEGA_MAX_RAD

    // ===== Yaw source (Pinpoint preferred) =====
    private DoubleSupplier yawDegSupplier; // returns degrees

    // ===== State =====
    private double yawOffset = 0;          // radians
    private double desiredPsi = 0;         // radians
    private double lastDesiredPsi = 0;     // radians
    private double sSinceDirChange = 0;    // meters
    private Pose   lastPose;

    // For velocity and spin estimation
    private double lastHeading;            // radians
    private long   lastKinematicsNs;       // nanoseconds

    // Two separate modes
    private boolean precisionMode = false; // LB
    private boolean slowMode      = false; // RB

    private boolean prevMoving = false;
    private boolean prevManualRot = false;
    private boolean inRest = false;
    private double  restArmingT = 0.0;     // seconds since entering rest
    private double  autoHeadingRearmT = 0; // seconds remaining until auto-heading allowed again

    // Slew limiting
    private double prevFx = 0, prevFy = 0;
    private long   lastInputTsNanos = 0;

    // ===== API for Auto to set yaw offset using known start heading =====
    public static void setYawOffsetFromAuto(FieldCentricAutoHeadingTeleOp teleOp, double fieldHeadingRad, double rawYawRad){
        teleOp.yawOffset = wrap(fieldHeadingRad - rawYawRad);
    }
    public static void setYawOffsetFromAutoDeg(FieldCentricAutoHeadingTeleOp teleOp, double fieldHeadingDeg, double rawYawDeg){
        setYawOffsetFromAuto(teleOp, Math.toRadians(fieldHeadingDeg), Math.toRadians(rawYawDeg));
    }

    // ===== Lifecycle =====
    @Override
    public void init() {
        dash = FtcDashboard.getInstance();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        lastPose = follower.getPose();
        lastHeading = follower.getHeading();
        lastKinematicsNs = System.nanoTime();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Default yaw provider uses Pedro heading
        yawDegSupplier = () -> Math.toDegrees(follower.getHeading());

        long now = System.nanoTime();
        lastInputTsNanos = now;
        prevFx = 0; prevFy = 0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        // If Auto did not set an offset, make current facing the field-forward 0 deg
        if (startingPose == null) setZeroToCurrent();

        desiredPsi = getFieldYaw();
        lastDesiredPsi = desiredPsi;
        sSinceDirChange = 0;

        // ====== Mode bindings ======
        // Slow Mode on RIGHT bumper (RB): suppress auto-heading + speed scaling
        button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(() -> {
                    slowMode = true;
                    desiredPsi = getFieldYaw();
                    lastDesiredPsi = desiredPsi;
                })
                .whenBecomesFalse(() -> {
                    slowMode = false;
                    desiredPsi = getFieldYaw();
                    lastDesiredPsi = desiredPsi;
                });

        // Precision Mode on LEFT bumper (LB): suppress auto-heading, no speed scaling
        button(() -> gamepad1.left_bumper)
                .whenBecomesTrue(() -> {
                    precisionMode = true;
                    desiredPsi = getFieldYaw();
                    lastDesiredPsi = desiredPsi;
                })
                .whenBecomesFalse(() -> {
                    precisionMode = false;
                    desiredPsi = getFieldYaw();
                    lastDesiredPsi = desiredPsi;
                });

        // Optional D-pad zeroing helpers
        button(() -> gamepad1.dpad_up)   .whenBecomesTrue(() -> setZeroToDeg(0));
        button(() -> gamepad1.dpad_left) .whenBecomesTrue(() -> setZeroToDeg(90));
        button(() -> gamepad1.dpad_down) .whenBecomesTrue(() -> setZeroToDeg(180));
        button(() -> gamepad1.dpad_right).whenBecomesTrue(() -> setZeroToDeg(270));
    }

    @Override
    public void loop() {
        // Update bindings first so they affect this loop
        BindingManager.update();

        follower.update();

        // === Publish pose to AdvantageScope via FTC Dashboard ===
        Pose p = follower.getPose();
        double xIn = p.getX();
        double yIn = p.getY();
        if (POSE_IS_METERS) { // set this flag if your pose is meters
            final double M_TO_IN = 39.37007874;
            xIn *= M_TO_IN;
            yIn *= M_TO_IN;
        }

        double headingRad = follower.getHeading();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Pose x", xIn);              // inches
        packet.put("Pose y", yIn);              // inches
        packet.put("Pose heading", headingRad); // radians
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.update();

        // Pedro field convention: X = forward (away from alliance wall), Y = left
        long nowInput = System.nanoTime();
        double dtIn = (nowInput - lastInputTsNanos) / 1e9; if (dtIn <= 0) dtIn = 1e-3;

        // Map sticks into FIELD frame
        double fx = -gamepad1.left_stick_y;    // forward +
        double fy =  gamepad1.left_stick_x;    // left +
        if (INVERT_FIELD_X) fx = -fx;
        if (INVERT_FIELD_Y) fy = -fy;

        // RAW right stick for override detection
        double rxRaw = INVERT_ROT_STICK ? gamepad1.right_stick_x : -gamepad1.right_stick_x;
        // Deadbanded right stick for actual rotation rate
        double rx = deadband(rxRaw, ROT_DEADBAND);

        fx = deadband(fx, MOVE_DEADBAND);
        fy = deadband(fy, MOVE_DEADBAND);

        // Slew limit left stick to avoid step changes
        fx = slew(fx, prevFx, SLEW_RATE_UNITS_PER_S, dtIn);
        fy = slew(fy, prevFy, SLEW_RATE_UNITS_PER_S, dtIn);
        prevFx = fx; prevFy = fy; lastInputTsNanos = nowInput;

        // Advance the re-arm timer
        autoHeadingRearmT = Math.max(0, autoHeadingRearmT - dtIn);

        // Current state
        double psi = getFieldYaw(); // current heading in radians
        double mag = Math.hypot(fx, fy);
        boolean isMoving = mag > MOVE_DEADBAND;
        boolean manualRot = Math.abs(rxRaw) > MANUAL_OVERRIDE_THRESH;

        // Combined “suppress auto-heading” flag from either mode
        boolean modeSuppress = slowMode || precisionMode;

        // Detect edges for rest logic
        boolean justStopped        =  prevMoving && !isMoving && !manualRot;
        boolean justReleasedRot    =  prevManualRot && !manualRot && !isMoving;
        boolean enteringRest       = (justStopped || justReleasedRot) || (!inRest && !isMoving && !manualRot);
        boolean leavingRest        =  inRest && (isMoving || manualRot);

        // Handle rest entry and exit
        if (enteringRest) {
            desiredPsi = psi;
            restArmingT = 0.0;
            inRest = true;
        } else if (leavingRest) {
            inRest = false;
        }
        if (inRest) restArmingT += dtIn;

        // Determine desired heading based on mode
        if (manualRot) {
            desiredPsi = psi;
            autoHeadingRearmT = AUTOHEADING_REARM_TIME_S;
        } else if (isMoving && !modeSuppress && autoHeadingRearmT == 0) {
            desiredPsi = Math.atan2(fy, fx);
        } else if (!isMoving && !manualRot) {
            double err = Math.toDegrees(Math.abs(wrap(desiredPsi - psi)));
            if (restArmingT >= REST_ARM_TIME_S && err >= DRIFT_TAKEOVER_DEG) {
                desiredPsi = psi;
            }
        }

        // Reset path distance if desired heading jumps
        double dPsiDes = wrap(desiredPsi - lastDesiredPsi);
        if (Math.abs(dPsiDes) > Math.toRadians(DIR_RESET_DEG)) sSinceDirChange = 0;
        sSinceDirChange += updateDistance();
        lastDesiredPsi = desiredPsi;

        // Angular command
        double e = wrap(desiredPsi - psi);
        double omega;
        if (manualRot){
            omega = rx * OMEGA_MAX_RAD; // manual only
        } else if (modeSuppress){
            omega = 0.0; // rotation suppressed in both modes
        } else if (isMoving){
            double v = mag; // proxy from stick magnitude
            omega = clamp(v * e / L_CONV_M, -OMEGA_MAX_RAD, OMEGA_MAX_RAD);
        } else {
            omega = 0.0;
        }

        // ===== Measured velocities (field frame) for active braking =====
        long nowKin = System.nanoTime();
        double dtKin = (nowKin - lastKinematicsNs) / 1e9; if (dtKin <= 0) dtKin = 1e-3;

        Pose pNow = follower.getPose();
        double vx = (pNow.getX() - lastPose.getX()) / dtKin;  // forward +, units per second
        double vy = (pNow.getY() - lastPose.getY()) / dtKin;  // left    +
        double headingNow = follower.getHeading();
        double omegaMeas = wrap(headingNow - lastHeading) / dtKin; // rad/s

        lastPose = pNow;
        lastHeading = headingNow;
        lastKinematicsNs = nowKin;

        // Speed multiplier: RB (slowMode) applies scaling; LB (precisionMode) does not
        double f = slowMode ? PRECISION_MULTIPLIER : 1.0;

        // ===== Active braking when the driver is not commanding =====
        boolean driverReleased = !isMoving && !manualRot;

        double fxBrake = 0, fyBrake = 0, omegaBrake = 0;

        if (ENABLE_ACTIVE_BRAKE && driverReleased) {
            double vMag = Math.hypot(vx, vy);
            if (vMag > VEL_DEADBAND_MPS) {
                // Oppose measured velocity. Units agnostic. Tune K_BRAKE empirically if pose is inches.
                fxBrake = clamp(-K_BRAKE * vx, -MAX_BRAKE_STICK, MAX_BRAKE_STICK);
                fyBrake = clamp(-K_BRAKE * vy, -MAX_BRAKE_STICK, MAX_BRAKE_STICK);
            }
        }

        if (ENABLE_ANGULAR_BRAKE && driverReleased && Math.abs(omegaMeas) > OMEGA_DEADBAND_RADPS) {
            // Map measured spin to a fraction of OMEGA_MAX_RAD, then oppose it
            double omegaFrac = clamp(-K_OMEGA_BRAKE * (omegaMeas / OMEGA_MAX_RAD), -1.0, 1.0);
            omegaBrake = omegaFrac * OMEGA_MAX_RAD;
        }

        // Compose final commands
        double fxCmd = (fx * f) + fxBrake;
        double fyCmd = (fy * f) + fyBrake;
        double omegaCmd = (omega * f) + omegaBrake;

        // Snap tiny values to exactly zero so ZeroPowerBehavior.BRAKE can engage
        fxCmd = snapZero(fxCmd, ZERO_SNAP);
        fyCmd = snapZero(fyCmd, ZERO_SNAP);
        omegaCmd = snapZero(omegaCmd, ZERO_SNAP);

        // Drive
        follower.setTeleOpDrive(fxCmd, fyCmd, omegaCmd, FIELD_OR_ROBOT_FLAG);

        // Bookkeeping
        prevMoving = isMoving;
        prevManualRot = manualRot;

        // Debug telemetry
        telemetryM.debug("PrecisionMode (LB)", precisionMode);
        telemetryM.debug("SlowMode (RB)", slowMode);
        telemetryM.debug("AutoHeadingSuppressed", modeSuppress);
        telemetryM.debug("AutoHeadingRearm(s)", autoHeadingRearmT);
        telemetryM.debug("InRest", inRest);
        telemetryM.debug("RestArmingT(s)", restArmingT);
        telemetryM.debug("Field Yaw (deg)", Math.toDegrees(psi));
        telemetryM.debug("Desired Yaw (deg)", Math.toDegrees(desiredPsi));
        telemetryM.debug("Heading Err (deg)", Math.toDegrees(e));
        telemetryM.debug("OmegaCmd (deg/s)", Math.toDegrees(omegaCmd));
        telemetryM.debug("fxCmd, fyCmd", String.format("%.3f, %.3f", fxCmd, fyCmd));
        telemetryM.debug("vx, vy (units/s)", String.format("%.3f, %.3f", vx, vy));
        telemetryM.debug("omegaMeas (deg/s)", Math.toDegrees(omegaMeas));
    }

    @Override
    public void stop() {
        BindingManager.reset();
    }

    // ===== Helpers =====
    private double getFieldYaw(){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        double psi = wrap(psiRaw + yawOffset);
        return NEGATE_FIELD_YAW ? -psi : psi;
    }

    private void setZeroToCurrent(){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(0 - psiRaw);
    }

    private double updateDistance(){
        Pose p = follower.getPose();
        double dx = p.getX() - lastPose.getX(); // field X forward
        double dy = p.getY() - lastPose.getY(); // field Y left
        lastPose = p;
        return Math.hypot(dx, dy);
    }

    private void setZeroToDeg(double deg){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(Math.toRadians(deg) - psiRaw);
        desiredPsi = getFieldYaw();
        lastDesiredPsi = desiredPsi;
    }

    private static double deadband(double v, double d){ return Math.abs(v) < d ? 0 : v; }
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private static double wrap(double a){
        while (a >  Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
    private static double slew(double target, double prev, double rate, double dt){
        double maxDelta = rate * dt;
        double delta = clamp(target - prev, -maxDelta, maxDelta);
        return prev + delta;
    }
    private static double snapZero(double v, double eps){ return Math.abs(v) < eps ? 0.0 : v; }

    // Optional: swap in Pinpoint degrees as the yaw source
    public void usePinpointYawDegrees(DoubleSupplier pinpointYawDegrees){
        this.yawDegSupplier = pinpointYawDegrees;
    }
}

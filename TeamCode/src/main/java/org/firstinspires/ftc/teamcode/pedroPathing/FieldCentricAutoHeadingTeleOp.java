//untested code with override for right stick
package org.firstinspires.ftc.teamcode.pedroPathing;

import static dev.nextftc.bindings.Bindings.button;

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
 * Single Precision Mode while RIGHT BUMPER is held:
 *   - Slower translation and rotation
 *   - Auto-heading suppressed for clean strafes and fine control
 * SDK: 11.0.0, PedroPathing: 2.0.2
 */
@Configurable
@TeleOp(name = "FieldCentricAutoHeadingTeleOp", group = "Pedro")
public class FieldCentricAutoHeadingTeleOp extends OpMode {

    // ===== Pedro follower and pose =====
    private Follower follower;
    public static Pose startingPose; // optional: set from Auto

    // ===== Telemetry =====
    private TelemetryManager telemetryM;

    // ===== Configurables (simple) =====
    public static double MOVE_DEADBAND = 0.08;                 // left stick
    public static double ROT_DEADBAND  = 0.10;                 // right stick
    public static double OMEGA_MAX_RAD = Math.toRadians(90);   // max yaw rate
    public static double KP_HOLD       = 1.4;                  // heading hold gain when stopped
    public static double L_CONV_M      = 2.5;                  // meters over which heading converges while moving
    public static double SLEW_RATE_UNITS_PER_S = 3.0;          // left-stick rate limit 0..1/s
    public static double DIR_RESET_DEG = 11;                   // reset threshold

    // Precision Mode scale while right bumper is held
    public static double PRECISION_MULTIPLIER = 0.40;          // scales both translation and rotation

    // After manual rotation ends, wait this long before re-enabling auto-heading
    public static double AUTOHEADING_REARM_TIME_S = 0.35;

    // Treat any right-stick input past this raw threshold as a manual override
    public static double MANUAL_OVERRIDE_THRESH = 0.02;

    // Final flag passed to setTeleOpDrive(..., field_or_robot_flag)
    // Set this to the value your Pedro version expects for FIELD-CENTRIC.
    public static boolean FIELD_OR_ROBOT_FLAG = false;

    // Axis and rotation sign flips
    public static boolean INVERT_FIELD_X = false;              // invert field X (forward)
    public static boolean INVERT_FIELD_Y = true;               // invert field Y (left)
    public static boolean INVERT_ROT_STICK = false;            // flip right stick rotation if needed
    public static boolean NEGATE_FIELD_YAW = false;            // negate psi if IMU heading sign is opposite

    // ===== Yaw source (Pinpoint preferred) =====
    private DoubleSupplier yawDegSupplier; // returns degrees

    // ===== State =====
    private double yawOffset = 0;          // radians
    private double desiredPsi = 0;         // radians
    private double lastDesiredPsi = 0;     // radians
    private double sSinceDirChange = 0;    // meters
    private Pose   lastPose;

    // Precision Mode flag (held while RB is pressed)
    private boolean precisionMode = false;

    // Manual rotation state
    private boolean prevManualRot = false; // detect override release
    private double autoHeadingRearmT = 0;  // seconds remaining until auto-heading is allowed again

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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        lastPose = follower.getPose();
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

        // Precision Mode while RIGHT BUMPER is held
        button(() -> gamepad1.right_bumper)
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

        double psi = getFieldYaw(); // radians

        // Manual override uses RAW threshold, so any small driver input disables auto-heading immediately
        boolean manualRot = Math.abs(rxRaw) > MANUAL_OVERRIDE_THRESH;

        // Auto-heading is allowed only if:
        //  - not in precision mode
        //  - not currently rotating manually
        //  - the re-arm timer has expired
        double mag = Math.hypot(fx, fy);
        boolean allowAutoHeading = !precisionMode && !manualRot && (autoHeadingRearmT == 0);

        if (mag > MOVE_DEADBAND && allowAutoHeading){
            desiredPsi = Math.atan2(fy, fx); // field direction
        }

        // Reset path distance if desired heading jumps
        double dPsiDes = wrap(desiredPsi - lastDesiredPsi);
        if (Math.abs(dPsiDes) > Math.toRadians(DIR_RESET_DEG)) sSinceDirChange = 0;
        sSinceDirChange += updateDistance();
        lastDesiredPsi = desiredPsi;

        // On manual rotation release, snap hold to current and start the re-arm delay
        if (prevManualRot && !manualRot) {
            desiredPsi = psi;
            lastDesiredPsi = desiredPsi;
            autoHeadingRearmT = AUTOHEADING_REARM_TIME_S;
        }
        prevManualRot = manualRot;

        // Angular command
        double e = wrap(desiredPsi - psi);
        double omega;
        if (manualRot){
            omega = rx * OMEGA_MAX_RAD; // manual override only, no correction
        } else if (mag <= MOVE_DEADBAND || precisionMode){
            omega = KP_HOLD * e;        // firm hold when stopped or in precision mode
        } else {
            double v = mag;             // proxy for speed from stick magnitude
            omega = clamp(v * e / L_CONV_M, -OMEGA_MAX_RAD, OMEGA_MAX_RAD);
        }

        // One slow factor, held on RB
        double f = precisionMode ? PRECISION_MULTIPLIER : 1.0;

        // Pass the flag exactly as your Pedro version expects
        follower.setTeleOpDrive(fx * f, fy * f, omega * f, FIELD_OR_ROBOT_FLAG);

        // Debug telemetry
        telemetryM.debug("PrecisionMode(RB held)", precisionMode);
        telemetryM.debug("AutoHeadingRearm(s)", autoHeadingRearmT);
        telemetryM.debug("Field Yaw (deg)", Math.toDegrees(psi));
        telemetryM.debug("Desired Yaw (deg)", Math.toDegrees(desiredPsi));
        telemetryM.debug("Omega (deg/s)", Math.toDegrees(omega));
        telemetryM.debug("fx, fy", String.format("%.3f, %.3f", fx, fy));
        telemetryM.debug("FlagPassed", FIELD_OR_ROBOT_FLAG);
        telemetryM.debug("InvertX,Y", String.format("%b,%b", INVERT_FIELD_X, INVERT_FIELD_Y));
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

    // Optional: swap in Pinpoint degrees as the yaw source
    public void usePinpointYawDegrees(DoubleSupplier pinpointYawDegrees){
        this.yawDegSupplier = pinpointYawDegrees;
    }
}

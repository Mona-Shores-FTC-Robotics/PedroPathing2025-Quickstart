// FieldCentricAutoHeadingTeleOp (student-friendly)
// - Field-centric mecanum
// - Auto-heading to left-stick direction
// - Right-stick = manual yaw override with re-arm delay
// - Precision Mode (hold RB): slows everything and holds current heading

package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.DoubleSupplier;

import dev.nextftc.bindings.BindingManager;

@Configurable
@TeleOp(name = "FieldCentricAutoHeadingTeleOp", group = "Pedro")
public class Teleop extends OpMode {

    // ─────────────────────────────────────────────────────────────────────────────
    // 1) Tunables (all knobs in one place)
    // ─────────────────────────────────────────────────────────────────────────────
    public static double MOVE_DEADBAND = 0.08;                 // left stick
    public static double ROT_DEADBAND  = 0.10;                 // right stick (used for actual command)
    public static double MANUAL_OVERRIDE_THRESH = 0.02;        // raw threshold to *detect* override
    public static double OMEGA_MAX_RAD = Math.toRadians(90);   // cap yaw rate (both manual & auto)
    public static double HOLD_KP       = 1.4;                  // heading hold when stopped / precision
    public static double CONVERGE_LEN_M = 2.5;                 // "meters" over which heading aligns
    public static double SLEW_RATE_PER_S = 3.0;                // rate limit for fx, fy (0..1 per second)
    public static double DIR_RESET_DEG  = 11;                  // if desired direction jumps > this, reset path distance
    public static double REARM_DELAY_S  = 0.35;                // wait after manual yaw ends before re-enabling auto
    public static double PRECISION_MULT = 0.40;                // scale for precision mode (right bumper)

    // Pedro flag for field vs robot frame (pass what your version expects for FIELD mode)
    public static boolean FIELD_OR_ROBOT_FLAG = false;

    // Axis and sign options (adjust once then forget)
    public static boolean INVERT_FIELD_X = false;              // forward +
    public static boolean INVERT_FIELD_Y = true;               // left +
    public static boolean INVERT_ROT_STICK = false;            // right stick x flip
    public static boolean NEGATE_FIELD_YAW = false;            // IMU sign mismatch helper

    // ─────────────────────────────────────────────────────────────────────────────
    // 2) Members
    // ─────────────────────────────────────────────────────────────────────────────
    private Follower follower;
    public static Pose startingPose; // optional: injected by Auto
    private TelemetryManager tel;
    private DoubleSupplier yawDeg;   // field yaw provider (degrees)

    // Heading state
    private double yawOffsetRad = 0;     // align IMU yaw to field zero
    private double desiredYaw = 0;       // target field yaw (rad)
    private double lastDesiredYaw = 0;
    private double distSinceDirChange = 0;
    private Pose   lastPose;

    // Driver state
    private boolean precisionMode = false;
    private boolean wasManualYaw = false;
    private double  rearmTimerS   = 0;

    // Slew limiting inputs
    private double prevFx = 0, prevFy = 0;
    private long   lastInputNanos = 0;

    // ─────────────────────────────────────────────────────────────────────────────
    // 3) Public helpers for Auto to set start heading
    // ─────────────────────────────────────────────────────────────────────────────
    public static void setYawOffsetFromAuto(Teleop teleOp, double fieldHeadingRad, double rawYawRad){
        teleOp.yawOffsetRad = wrap(fieldHeadingRad - rawYawRad);
    }
    public static void setYawOffsetFromAutoDeg(Teleop teleOp, double fieldHeadingDeg, double rawYawDeg){
        setYawOffsetFromAuto(teleOp, Math.toRadians(fieldHeadingDeg), Math.toRadians(rawYawDeg));
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // 4) Lifecycle
    // ─────────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        lastPose = follower.getPose();

        tel = PanelsTelemetry.INSTANCE.getTelemetry();

        // Default yaw provider: Pedro follower heading (radians → degrees)
        yawDeg = () -> Math.toDegrees(follower.getHeading());

        lastInputNanos = System.nanoTime();
        prevFx = 0; prevFy = 0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        // If Auto didn't seed, make current facing = field-forward(0 deg)
        if (startingPose == null) zeroFieldYawToCurrent();

        desiredYaw = getFieldYaw();
        lastDesiredYaw = desiredYaw;
        distSinceDirChange = 0;

        // Precision Mode (RIGHT BUMPER)
        button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(() -> {
                    precisionMode = true;
                    snapDesiredToCurrent();
                })
                .whenBecomesFalse(() -> {
                    precisionMode = false;
                    snapDesiredToCurrent();
                });

        // Quick set field zero (D-pad)
        button(() -> gamepad1.dpad_up)   .whenBecomesTrue(() -> setFieldZeroDeg(0));
        button(() -> gamepad1.dpad_left) .whenBecomesTrue(() -> setFieldZeroDeg(90));
        button(() -> gamepad1.dpad_down) .whenBecomesTrue(() -> setFieldZeroDeg(180));
        button(() -> gamepad1.dpad_right).whenBecomesTrue(() -> setFieldZeroDeg(270));
    }

    @Override
    public void loop() {
        // Keep bindings fresh
        BindingManager.update();

        // Pedro & Telemetry
        follower.update();
        tel.update();

        // ── A) Read & condition inputs
        double dtIn = secondsSinceLastInputs();
        Sticks s = readSticks(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        s.applyDeadbands(MOVE_DEADBAND, ROT_DEADBAND);
        s.applyAxisInverts(INVERT_FIELD_X, INVERT_FIELD_Y, INVERT_ROT_STICK);

        // Slew-limit translation to avoid step jumps
        s.fx = slew(s.fx, prevFx, SLEW_RATE_PER_S, dtIn);
        s.fy = slew(s.fy, prevFy, SLEW_RATE_PER_S, dtIn);
        prevFx = s.fx; prevFy = s.fy;

        // Rearm countdown
        rearmTimerS = Math.max(0, rearmTimerS - dtIn);

        // ── B) Update auto-heading state
        double psi = getFieldYaw();                         // current field yaw (rad)
        boolean manualYaw = Math.abs(s.rxRaw) > MANUAL_OVERRIDE_THRESH;
        boolean canAuto = !precisionMode && !manualYaw && (rearmTimerS == 0);
        double mag = Math.hypot(s.fx, s.fy);

        if (mag > MOVE_DEADBAND && canAuto) {
            desiredYaw = Math.atan2(s.fy, s.fx);           // point where we’re driving
        }

        // If desired jump is big, reset "distance since direction change"
        double dPsiDes = wrap(desiredYaw - lastDesiredYaw);
        if (Math.abs(dPsiDes) > Math.toRadians(DIR_RESET_DEG)) distSinceDirChange = 0;
        distSinceDirChange += distanceSinceLastPose();
        lastDesiredYaw = desiredYaw;

        // On manual-yaw release: hold current yaw and start re-arm timer
        if (wasManualYaw && !manualYaw) {
            desiredYaw = psi;
            lastDesiredYaw = desiredYaw;
            rearmTimerS = REARM_DELAY_S;
        }
        wasManualYaw = manualYaw;

        // ── C) Compute yaw command
        double yawErr = wrap(desiredYaw - psi);
        double omega;
        if (manualYaw) {
            omega = s.rx * OMEGA_MAX_RAD;                  // direct manual control
        } else if (mag <= MOVE_DEADBAND || precisionMode) {
            omega = HOLD_KP * yawErr;                      // “stiff” hold when stopped/precision
        } else {
            // “Move = converge toward desired” scaled by speed proxy (stick magnitude)
            double v = mag;
            omega = clamp(v * yawErr / CONVERGE_LEN_M, -OMEGA_MAX_RAD, OMEGA_MAX_RAD);
        }

        // ── D) Precision scale and drive
        double scale = precisionMode ? PRECISION_MULT : 1.0;
        follower.setTeleOpDrive(s.fx * scale, s.fy * scale, omega * scale, FIELD_OR_ROBOT_FLAG);

        // ── E) Debug telemetry (short, consistent names)
        tel.debug("PrecMode(RB)", precisionMode);
        tel.debug("Rearm(s)", rearmTimerS);
        tel.debug("Yaw(deg)", Math.toDegrees(psi));
        tel.debug("YawDes(deg)", Math.toDegrees(desiredYaw));
        tel.debug("Omega(deg/s)", Math.toDegrees(omega));
        tel.debug("fx,fy", String.format("%.3f, %.3f", s.fx, s.fy));
        tel.debug("Flag", FIELD_OR_ROBOT_FLAG);
        tel.debug("InvertX,Y", String.format("%b,%b", INVERT_FIELD_X, INVERT_FIELD_Y));
    }

    @Override
    public void stop() {
        BindingManager.reset();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // 5) Small, named helpers (easy to teach/trace)
    // ─────────────────────────────────────────────────────────────────────────────
    private void snapDesiredToCurrent() {
        desiredYaw = getFieldYaw();
        lastDesiredYaw = desiredYaw;
    }

    private double getFieldYaw() {
        double psiRaw = Math.toRadians(yawDeg.getAsDouble());
        double psi = wrap(psiRaw + yawOffsetRad);
        return NEGATE_FIELD_YAW ? -psi : psi;
    }

    private void zeroFieldYawToCurrent() {
        double psiRaw = Math.toRadians(yawDeg.getAsDouble());
        yawOffsetRad = wrap(0 - psiRaw);
    }

    private void setFieldZeroDeg(double deg) {
        double psiRaw = Math.toRadians(yawDeg.getAsDouble());
        yawOffsetRad = wrap(Math.toRadians(deg) - psiRaw);
        snapDesiredToCurrent();
    }

    private double distanceSinceLastPose() {
        Pose p = follower.getPose();
        double dx = p.getX() - lastPose.getX(); // field X forward
        double dy = p.getY() - lastPose.getY(); // field Y left
        lastPose = p;
        return Math.hypot(dx, dy);
    }

    private double secondsSinceLastInputs() {
        long now = System.nanoTime();
        double dt = (now - lastInputNanos) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastInputNanos = now;
        return dt;
    }

    // Optional: swap in Pinpoint degrees as yaw source
    public void usePinpointYawDegrees(DoubleSupplier pinpointYawDegrees) {
        this.yawDeg = pinpointYawDegrees;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // 6) Tiny data class for sticks (keeps loop() clean)
    // ─────────────────────────────────────────────────────────────────────────────
    private static class Sticks {
        // Field-frame translation (fx forward +, fy left +)
        double fx, fy;

        // Right-stick rotation (two versions)
        double rxRaw;  // raw for override detection
        double rx;     // deadbanded/scaled for actual omega

        Sticks(double fx, double fy, double rxRaw) {
            this.fx = fx;
            this.fy = fy;
            this.rxRaw = rxRaw;
            this.rx = rxRaw; // will be deadbanded later
        }

        void applyDeadbands(double moveDb, double rotDb) {
            fx = deadband(fx, moveDb);
            fy = deadband(fy, moveDb);
            rx = deadband(rx, rotDb);
        }

        void applyAxisInverts(boolean invX, boolean invY, boolean invertRot) {
            if (invX) fx = -fx;
            if (invY) fy = -fy;
            if (invertRot) { rxRaw = -rxRaw; rx = -rx; }
        }
    }

    private Sticks readSticks(double lx, double ly, double rx) {
        // FTC: left_y is +down, so negate to make forward +
        double fx = -ly;        // forward +
        double fy =  lx;        // left +
        double rxRaw = -rx;     // positive = CCW (standardize)
        return new Sticks(fx, fy, rxRaw);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // 7) Math utilities (self-contained for students)
    // ─────────────────────────────────────────────────────────────────────────────
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
}

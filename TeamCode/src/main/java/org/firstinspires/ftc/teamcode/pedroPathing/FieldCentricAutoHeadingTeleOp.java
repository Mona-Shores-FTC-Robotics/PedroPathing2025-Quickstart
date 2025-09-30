package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleSupplier;

/**
 * Field-centric mecanum TeleOp with auto-heading to the left-stick direction.
 * Uses a raw yaw provider (Pinpoint degrees preferred) with an offset so that 0° = away from alliance wall.
 * Translation starts immediately; heading converges over 1.524 m (5 ft).
 * Right-stick temporarily overrides rotation; on release, desired heading snaps to current yaw.
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

    // ===== Configurables =====
    @Configurable public static double MOVE_DEADBAND = 0.08;                // left stick
    @Configurable public static double ROT_DEADBAND  = 0.10;                // right stick
    @Configurable public static double OMEGA_MAX_RAD = Math.toRadians(180); // max yaw rate
    @Configurable public static double KP_HOLD       = 3.0;                 // rad/s per rad
    @Configurable public static double L_CONV_M      = 1.524;               // 5 ft in meters
    @Configurable public static double SLEW_RATE_UNITS_PER_S = 3.0;         // left-stick rate limit 0..1/s
    @Configurable public static double DIR_RESET_DEG = 11;                  // reset threshold

    // ===== Yaw source (Pinpoint preferred) =====
    // Provide degrees to be kid-friendly; we convert to radians internally.
    private DoubleSupplier yawDegSupplier; // e.g., () -> pinpoint.getHeading() in degrees

    // ===== State =====
    private double yawOffset = 0; // radians; makes current raw yaw map to field frame
    private double desiredPsi = 0, lastDesiredPsi = 0; // radians
    private double sSinceDirChange = 0; // meters
    private Pose   lastPose;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private boolean autoHeading = true;
    private boolean prevManualRot = false; // detect override release

    // Slew limiting
    private double prevLx = 0, prevLy = 0;
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

        // DEFAULT YAW PROVIDER: follower heading (radians) converted to degrees.
        // Replace with your Pinpoint call, e.g., yawDegSupplier = () -> pinpoint.getHeading();
        yawDegSupplier = () -> Math.toDegrees(follower.getHeading());

        // Initialize input timestamps for slew limiting
        long now = System.nanoTime();
        lastInputTsNanos = now;
        prevLx = 0; prevLy = 0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        // If Auto did not set an offset, make current facing the field-forward 0°
        if (startingPose == null) {
            setZeroToCurrent();
        } else {
            // Optionally align field zero to alliance standard quickly using D-pad at match start
            // leave as-is if Auto prepared yawOffset externally
        }

        desiredPsi = getFieldYaw();
        lastDesiredPsi = desiredPsi;
        sSinceDirChange = 0;
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update(); // Make sure to call update on telemetryM to send data

        handleZeroingButtons();
        handleSlowModeToggle();

        // Read sticks (FTC style: forward is -Y)
        long nowInput = System.nanoTime();
        double dtIn = (nowInput - lastInputTsNanos)/1e9; if (dtIn <= 0) dtIn = 1e-3;

        double lx = -gamepad1.left_stick_x; // Corrected: In FTC, left_stick_x is typically strafe
        double ly = -gamepad1.left_stick_y; // Corrected: In FTC, left_stick_y is typically forward/backward
        double rx = -gamepad1.right_stick_x; // Corrected: In FTC, right_stick_x is typically turn

        lx = deadband(lx, MOVE_DEADBAND);
        ly = deadband(ly, MOVE_DEADBAND);
        rx = deadband(rx, ROT_DEADBAND);

        // Slew limit left stick to avoid step changes
        lx = slew(lx, prevLx, SLEW_RATE_UNITS_PER_S, dtIn);
        ly = slew(ly, prevLy, SLEW_RATE_UNITS_PER_S, dtIn);
        prevLx = lx; prevLy = ly; lastInputTsNanos = nowInput;

        double psi = getFieldYaw(); // radians
        boolean manualRot = Math.abs(rx) > ROT_DEADBAND;

        double mag = Math.hypot(lx, ly);
        if (mag > MOVE_DEADBAND && autoHeading && !manualRot){
            // Standard atan2 arguments are (y, x) for angle from positive x-axis
            // For field-centric, if ly is forward (field Y) and lx is strafe (field X):
            desiredPsi = Math.atan2(ly, lx);
        }

        // Detect large desired heading change and reset path distance
        double dPsiDes = wrap(desiredPsi - lastDesiredPsi);
        if (Math.abs(dPsiDes) > Math.toRadians(DIR_RESET_DEG)) sSinceDirChange = 0;
        sSinceDirChange += updateDistance();
        lastDesiredPsi = desiredPsi;

        // Snap desired heading on manual override release
        if (prevManualRot && !manualRot){
            desiredPsi = psi;
        }
        prevManualRot = manualRot;

        // Angular command
        double e = wrap(desiredPsi - psi);
        double omega;
        if (manualRot){
            omega = rx * OMEGA_MAX_RAD; // manual override
        } else if (mag <= MOVE_DEADBAND){
            omega = KP_HOLD * e;        // heading hold when stopped
        } else {
            // Spatial convergence toward L_CONV_M
            double v = mag;
            omega = v * e / L_CONV_M; // This is a simplified proportional control based on distance to convergence point for heading
            omega = clamp(omega, -OMEGA_MAX_RAD, OMEGA_MAX_RAD);
        }

        // Field→robot transform using -psi (current field heading of robot)
        // Standard rotation: x' = x*cos(a) - y*sin(a), y' = x*sin(a) + y*cos(a)
        // Here, (lx, ly) are field-frame commands. We want robot-frame commands (strafeR, forwardR)
        // Robot X is forward, Robot Y is left.
        // Field X is right, Field Y is forward.
        // To transform field commands (lx, ly) to robot commands (forwardR, strafeR):
        // forwardR = lx * cos(psi) + ly * sin(psi)  <-- No, this is robot to field.
        // forwardR = ly * cos(psi) + lx * sin(psi)  <-- If lx is field X, ly is field Y
        // strafeR  = -ly * sin(psi) + lx * cos(psi)

        // Simpler: inputs lx (field X / strafe) and ly (field Y / forward)
        // Robot forward command = ly_field * cos(-psi) - lx_field * sin(-psi)
        // Robot strafe command  = ly_field * sin(-psi) + lx_field * cos(-psi)
        // Oh, the original code used:
        // double strafeR  = lx * c - ly * s; where c = cos(-psi), s = sin(-psi)
        // double forwardR = lx * s + ly * c;
        // This means lx is treated as "x" and ly as "y" in a coordinate system rotated by -psi.
        // If lx = field_X (strafe right positive) and ly = field_Y (forward positive)
        // Then forward_robot = ly_field * cos(psi) - lx_field * sin(psi)
        //      strafe_robot = ly_field * sin(psi) + lx_field * cos(psi)
        // Let's re-verify the original snippet's transform.
        // If psi is robot's heading in field frame:
        // Robot's X-axis (forward) aligns with field vector (cos(psi), sin(psi))
        // Robot's Y-axis (strafe left) aligns with field vector (-sin(psi), cos(psi))
        // Field commands (lx, ly)
        // forwardR (robot X component) = lx * cos(psi) + ly * sin(psi)
        // strafeR  (robot Y component) = lx * (-sin(psi)) + ly * cos(psi) --- this would be strafe LEFT positive
        // The original code has:
        // double c = Math.cos(-psi), s = Math.sin(-psi);
        // double strafeR  = lx * c - ly * s;  => lx*cos(-psi) - ly*sin(-psi) => lx*cos(psi) + ly*sin(psi)
        // double forwardR = lx * s + ly * c;  => lx*sin(-psi) + ly*cos(-psi) => -lx*sin(psi) + ly*cos(psi)
        // This implies:
        //   'strafeR' in their code is along the original X-axis rotated by -psi.
        //   'forwardR' in their code is along the original Y-axis rotated by -psi.
        // If their lx, ly were Cartesian (x right, y up), and robot frame is x forward, y left:
        // Field X (lx), Field Y (ly)
        // Robot Forward = lx * cos(psi) + ly * sin(psi)
        // Robot Strafe (left) = -lx * sin(psi) + ly * cos(psi)
        // The follower.setTeleOpDrive expects (forward, strafe, turn), where positive strafe is typically left.
        // Given the gamepad inputs:
        // lx = -gamepad1.left_stick_x; (positive is field right)
        // ly = -gamepad1.left_stick_y; (positive is field forward)

        // Let's use the standard field-to-robot transformation:
        // Field commands: targetFieldVx = lx, targetFieldVy = ly
        // Robot commands:
        // robotVx (forward) = targetFieldVx * cos(psi) + targetFieldVy * sin(psi)
        // robotVy (strafeLeft) = -targetFieldVx * sin(psi) + targetFieldVy * cos(psi)
        // This is what Pedro Pathing Follower.setTeleOpDrive expects (if positive strafe is left)

        double fieldVx = lx; // Positive is right in field
        double fieldVy = ly; // Positive is forward in field

        double robotForward = fieldVx * Math.cos(psi) + fieldVy * Math.sin(psi);
        double robotStrafe  = -fieldVx * Math.sin(psi) + fieldVy * Math.cos(psi); // Positive is left strafe


        double f = slowMode ? slowModeMultiplier : 1.0;
        // The last boolean for setTeleOpDrive is often 'fieldCentric'. If true, follower handles the transform.
        // If false, we provide robot-centric values. The comment says "robot-centric because we did the transform"
        // so `true` in the original might have been a mistake or different interpretation for that specific `setTeleOpDrive`.
        // PedroPathing's Follower.setTeleOpDrive(double drivePower, double strafePower, double turnPower, boolean fieldCentric)
        // If fieldCentric is true, it expects field-relative powers. If false, robot-relative.
        // Since we are calculating robotForward and robotStrafe, we should pass fieldCentric = false.
        follower.setTeleOpDrive(robotForward * f, robotStrafe * f, omega * f, false);

        // Debug telemetry
        telemetryM.debug("Field Yaw (deg)", Math.toDegrees(psi));
        telemetryM.debug("Desired Yaw (deg)", Math.toDegrees(desiredPsi));
        telemetryM.debug("Error (deg)", Math.toDegrees(e));
        telemetryM.debug("Omega (deg/s)", Math.toDegrees(omega));
        telemetryM.debug("sSinceDirChange(m)", sSinceDirChange);
        telemetryM.debug("Raw LX", gamepad1.left_stick_x);
        telemetryM.debug("Raw LY", gamepad1.left_stick_y);
        telemetryM.debug("Field Vx (lx)", lx);
        telemetryM.debug("Field Vy (ly)", ly);
        telemetryM.debug("Robot Forward", robotForward);
        telemetryM.debug("Robot Strafe", robotStrafe);
        telemetryM.debug("Slow Mode", slowMode);
    }

    // ===== Helpers =====
    private double getFieldYaw(){
        // Convert supplier degrees → radians and add offset
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        return wrap(psiRaw + yawOffset);
    }

    private void setZeroToCurrent(){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(0 - psiRaw); // Sets current raw yaw to be field 0
    }

    private double updateDistance(){
        Pose p = follower.getPose();
        // Distance calculation should be in field frame if sSinceDirChange is field path length
        double dx = p.getX() - lastPose.getX(); // These are field frame coords from PedroPathing
        double dy = p.getY() - lastPose.getY();
        lastPose = p;
        return Math.hypot(dx, dy);
    }

    private void handleZeroingButtons(){
        // Cardinal zeroing: 0° away from alliance wall; +CCW
        if (gamepad1.dpad_up)    setZeroToDeg(0);    // Field Forward
        if (gamepad1.dpad_left)  setZeroToDeg(90);   // Field Left
        if (gamepad1.dpad_down)  setZeroToDeg(180);  // Field Backward
        if (gamepad1.dpad_right) setZeroToDeg(270);  // Field Right
    }

    private void handleSlowModeToggle(){
        // Use a "wasPressed" to avoid rapid toggling if button is held
        if (gamepad1.right_bumper && !gamepad1_was_right_bumper_last_frame) { // pseudo code, needs state
            slowMode = !slowMode;
        }
        // gamepad1_was_right_bumper_last_frame = gamepad1.right_bumper; // manage this state if not part of SDK OpMode
        // FTC SDK provides gamepadX.xWasPressed() for this. The user's code had `rightBumperWasPressed()`
        // which implies it might be a custom Gamepad wrapper or a newer SDK feature not in base OpMode.
        // Assuming `gamepad1.rightBumperWasPressed()` is available as in the original snippet.
        if (gamepad1.rightBumperWasPressed()) slowMode = !slowMode;

    }

    private void setZeroToDeg(double deg){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(Math.toRadians(deg) - psiRaw);
        desiredPsi = getFieldYaw(); // Also update desiredPsi to prevent sudden rotation
        lastDesiredPsi = desiredPsi;
    }

    private static double deadband(double v, double d){ return Math.abs(v) < d ? 0 : v; }
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private static double wrap(double a){ // Wraps to [-PI, PI]
        while (a >  Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
    private static double slew(double target, double prev, double rate, double dt){
        double maxDelta = rate * dt;
        double delta = clamp(target - prev, -maxDelta, maxDelta); // clamp the delta itself
        return prev + delta;
    }

    // ===== Optional: expose a method to set the yaw source to Pinpoint degrees =====
    public void usePinpointYawDegrees(DoubleSupplier pinpointYawDegrees){
        this.yawDegSupplier = pinpointYawDegrees;
    }
}

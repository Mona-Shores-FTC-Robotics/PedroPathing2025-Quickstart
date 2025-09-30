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
 * Translation starts immediately; heading converges over L_CONV_M distance.
 * Right-stick temporarily overrides rotation; on release, desired heading snaps to current yaw.
 * SDK: 11.0.0, PedroPathing: 2.0.2
 */
@Configurable
@TeleOp(name = "FieldCentricAutoHeadingTeleOp", group = "Pedro")
public class Field2 extends OpMode {

    // ===== Pedro follower and pose =====
    private Follower follower;
    public static Pose startingPose; // optional: set from Auto

    // ===== Telemetry =====
    private TelemetryManager telemetryM;

    // ===== Configurables =====
     public static double MOVE_DEADBAND = 0.08;                // left stick
     public static double ROT_DEADBAND  = 0.10;                // right stick
     public static double OMEGA_MAX_RAD = Math.toRadians(180); // max yaw rate deg/s -> rad/s
     public static double KP_HOLD       = 3.0;                 // rad/s per rad error for heading hold
     public static double L_CONV_M      = 1.524;               // 5 ft in meters, convergence distance for auto-heading
     public static double SLEW_RATE_UNITS_PER_S = 3.0;         // left-stick rate limit (0..1 input units per second)
     public static double DIR_RESET_DEG = 11;                  // threshold in degrees for desired heading change to reset convergence distance

    // ===== Yaw source (Pinpoint preferred) =====
    private DoubleSupplier yawDegSupplier; // e.g., () -> pinpoint.getHeading() in degrees

    // ===== State =====
    private double yawOffset = 0; // radians; makes current raw yaw map to field frame (0 = field forward)
    private double desiredPsi = 0, lastDesiredPsi = 0; // radians, field frame
    private double sSinceDirChange = 0; // meters, path length since last significant direction change
    private Pose   lastPose;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5; // Speed multiplier for slow mode
    private boolean autoHeading = true;      // Whether auto-heading is active
    private boolean prevManualRot = false;   // Detects release of manual rotation override

    // Slew limiting state
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
        follower.update(); // Initialize pose
        lastPose = follower.getPose();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // DEFAULT YAW PROVIDER: follower heading (radians) converted to degrees.
        // REPLACE with your Pinpoint call or other yaw source, e.g.:
        // yawDegSupplier = () -> pinpoint.getHeading(); // Assuming pinpoint object exists and returns degrees
        yawDegSupplier = () -> Math.toDegrees(follower.getHeading()); // Uses PedroPathing's heading if no other source

        long now = System.nanoTime();
        lastInputTsNanos = now;
        prevLx = 0; prevLy = 0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive(); // Enables teleop mode for the follower

        if (startingPose == null) { // If not set by Auto
            setZeroToCurrent(); // Make current robot orientation field forward (0 degrees)
        }
        // If startingPose was set by Auto, yawOffset should ideally be set there too via setYawOffsetFromAutoDeg

        desiredPsi = getFieldYaw(); // Initialize desired heading to current field yaw
        lastDesiredPsi = desiredPsi;
        sSinceDirChange = 0;
    }

    @Override
    public void loop() {
        follower.update(); // Update follower and robot pose
        telemetryM.update(); // Send telemetry data to Panels

        handleZeroingButtons();
        handleSlowModeToggle();

        long nowInput = System.nanoTime();
        double dtIn = (nowInput - lastInputTsNanos) / 1.0e9;
        if (dtIn <= 0) dtIn = 1e-3; // Prevent division by zero or negative dt

        // Get gamepad inputs (FTC style: forward Y is negative)
        double rawLx = -gamepad1.left_stick_x; // Field X command (right positive)
        double rawLy = -gamepad1.left_stick_y; // Field Y command (forward positive)
        double rawRx = -gamepad1.right_stick_x; // Rotational command (CCW positive for omega)

        // Apply deadbands
        double lx = deadband(rawLx, MOVE_DEADBAND);
        double ly = deadband(rawLy, MOVE_DEADBAND);
        double rx = deadband(rawRx, ROT_DEADBAND);

        // Apply slew rate limiting to translational inputs
        lx = slew(lx, prevLx, SLEW_RATE_UNITS_PER_S, dtIn);
        ly = slew(ly, prevLy, SLEW_RATE_UNITS_PER_S, dtIn);
        prevLx = lx; prevLy = ly; lastInputTsNanos = nowInput;

        double currentFieldPsi = getFieldYaw(); // Robot's current heading in field frame (radians)
        boolean manualRotActive = Math.abs(rx) > ROT_DEADBAND; // True if right stick is actively used

        double translationMagnitude = Math.hypot(lx, ly);

        // Auto-heading: Update desiredPsi if translating and not manually rotating
        if (translationMagnitude > MOVE_DEADBAND && autoHeading && !manualRotActive) {
            desiredPsi = Math.atan2(ly, lx); // Angle of left stick vector in field frame
        }

        // Reset convergence distance if desired heading changes significantly
        double deltaDesiredPsi = wrap(desiredPsi - lastDesiredPsi);
        if (Math.abs(deltaDesiredPsi) > Math.toRadians(DIR_RESET_DEG)) {
            sSinceDirChange = 0;
        }
        sSinceDirChange += updateFieldDistanceTraveled(); // Accumulate distance traveled in field frame
        lastDesiredPsi = desiredPsi;

        // Snap desired heading to current on manual rotation release
        if (prevManualRot && !manualRotActive) {
            desiredPsi = currentFieldPsi;
        }
        prevManualRot = manualRotActive;

        // Calculate angular velocity (omega)
        double headingError = wrap(desiredPsi - currentFieldPsi);
        double omega;
        if (manualRotActive) {
            omega = rx * OMEGA_MAX_RAD; // Manual rotation override
        } else if (translationMagnitude <= MOVE_DEADBAND) {
            omega = KP_HOLD * headingError; // Hold heading when stationary
        } else {
            // Auto-heading: Proportional control based on heading error and convergence distance
            // As robot moves (sSinceDirChange increases), omega's response to error can be tuned.
            // This version uses a spatial convergence idea: error / (remaining_distance_to_converge)
            // A simpler approach could be just Kp_auto_heading * headingError
            double convergenceFactor = clamp(sSinceDirChange / L_CONV_M, 0.0, 1.0);
            omega = (KP_HOLD * headingError) * (1.0 - convergenceFactor) + (translationMagnitude * headingError / L_CONV_M) * convergenceFactor;
            // Fallback if L_CONV_M is too small or sSinceDirChange is 0, use simpler proportional control
            if (L_CONV_M == 0) omega = KP_HOLD * headingError;
        }
        omega = clamp(omega, -OMEGA_MAX_RAD, OMEGA_MAX_RAD);

        double speedFactor = slowMode ? slowModeMultiplier : 1.0;

        // Send field-centric commands to PedroPathing follower
        // lx is field X power (right), ly is field Y power (forward)
        follower.setTeleOpDrive(lx * speedFactor, ly * speedFactor, omega * speedFactor, true);

        // Telemetry for debugging
        telemetryM.debug("Field Yaw (deg)", Math.toDegrees(currentFieldPsi));
        telemetryM.debug("Desired Yaw (deg)", Math.toDegrees(desiredPsi));
        telemetryM.debug("Heading Error (deg)", Math.toDegrees(headingError));
        telemetryM.debug("Omega (deg/s)", Math.toDegrees(omega));
        telemetryM.debug("sSinceDirChange (m)", sSinceDirChange);
        telemetryM.debug("Field Cmd X (lx)", lx * speedFactor);
        telemetryM.debug("Field Cmd Y (ly)", ly * speedFactor);
        telemetryM.debug("Slow Mode", slowMode);
        telemetryM.debug("Raw Yaw Deg", yawDegSupplier.getAsDouble());
        telemetryM.debug("Yaw Offset Deg", Math.toDegrees(yawOffset));
    }

    // ===== Helper Methods =====
    private double getFieldYaw() {
        double rawYawRad = Math.toRadians(yawDegSupplier.getAsDouble());
        return wrap(rawYawRad + yawOffset);
    }

    private void setZeroToCurrent() {
        double rawYawRad = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(0 - rawYawRad); // Set current raw yaw to map to 0 field heading
        desiredPsi = getFieldYaw(); // Update desiredPsi to current after zeroing
        lastDesiredPsi = desiredPsi;
    }

    private double updateFieldDistanceTraveled() {
        Pose currentPose = follower.getPose();
        double dx = currentPose.getX() - lastPose.getX(); // Delta X in field frame
        double dy = currentPose.getY() - lastPose.getY(); // Delta Y in field frame
        lastPose = currentPose; // Update lastPose for next iteration
        return Math.hypot(dx, dy); // Distance traveled in field frame
    }

    private void handleZeroingButtons() {
        if (gamepad1.dpad_up)    setZeroToFieldAngleDeg(0);    // Robot front towards field positive Y
        if (gamepad1.dpad_left)  setZeroToFieldAngleDeg(90);   // Robot front towards field positive X
        if (gamepad1.dpad_down)  setZeroToFieldAngleDeg(180);  // Robot front towards field negative Y
        if (gamepad1.dpad_right) setZeroToFieldAngleDeg(270);  // Robot front towards field negative X
    }

    private void handleSlowModeToggle() {
        if (gamepad1.right_bumper && !gamepad1_was_right_bumper_last_frame) {
            slowMode = !slowMode;
        }
        gamepad1_was_right_bumper_last_frame = gamepad1.right_bumper;
    }
    // Need to manage this state if Gamepad.wasPressed() isn't directly available
    private boolean gamepad1_was_right_bumper_last_frame = false;

    private void setZeroToFieldAngleDeg(double fieldAngleDeg) {
        double rawYawRad = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(Math.toRadians(fieldAngleDeg) - rawYawRad);
        desiredPsi = getFieldYaw(); // Update desiredPsi to current field angle after zeroing
        lastDesiredPsi = desiredPsi;
        sSinceDirChange = 0; // Reset convergence path as we've re-oriented
    }

    private static double deadband(double value, double band) {
        return Math.abs(value) > band ? value : 0.0;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double wrap(double angleRad) { // Wraps angle to [-PI, PI]
        while (angleRad > Math.PI) angleRad -= 2 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2 * Math.PI;
        return angleRad;
    }

    private static double slew(double targetValue, double previousValue, double maxRateOfChange, double deltaTimeSec) {
        double maxChange = maxRateOfChange * deltaTimeSec;
        double delta = clamp(targetValue - previousValue, -maxChange, maxChange);
        return previousValue + delta;
    }

    // Optional: expose a method to set the yaw source, e.g., from Pinpoint
    public void usePinpointYawDegrees(DoubleSupplier pinpointYawDegreesSupplier) {
        this.yawDegSupplier = pinpointYawDegreesSupplier;
    }
}

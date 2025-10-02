package org.firstinspires.ftc.teamcode.pedroPathing;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;

import java.util.function.DoubleSupplier;

@Configurable
@TeleOp(name = "FieldCentricSpeedControl", group = "Pedro")
public class FieldCentricSpeedControl extends OpMode {

    // Pedro follower
    private Follower follower;
    public static Pose startingPose;

    // Telemetry
    private TelemetryManager telemetryM;

    // Configurables related to heading behavior
    public static double MOVE_DEADBAND = 0.08;
    public static double ROT_DEADBAND  = 0.10;
    public static double OMEGA_MAX_RAD = Math.toRadians(180);
    public static double KP_HOLD       = 3.0;
    public static double L_CONV_M      = 1.524;
    public static double SLEW_RATE_UNITS_PER_S = 3.0;
    public static double DIR_RESET_DEG = 11;

    // Yaw source from follower (Pinpoint under the hood)
    private DoubleSupplier yawDegSupplier;

    // State
    private double yawOffset = 0;
    private double desiredPsi = 0, lastDesiredPsi = 0;
    private double sSinceDirChange = 0;
    private Pose lastPose;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private boolean autoHeading = true;
    private boolean prevManualRot = false;

    // Input slew
    private double prevLx = 0, prevLy = 0;
    private long lastInputTsNanos = 0;

    // Speed mode toggle and slew state
    private boolean speedMode = false;
    private double prevVxCmd = 0.0, prevVyCmd = 0.0, prevWzCmd = 0.0;

    // Motors for speed mode
    private Constants.Motors motors;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        lastPose = follower.getPose();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        yawDegSupplier = () -> Math.toDegrees(follower.getHeading());

        motors = new Constants.Motors(hardwareMap);
        motors.setRunUsingEncoder();

        long now = System.nanoTime();
        lastInputTsNanos = now;
        prevLx = 0; prevLy = 0;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        if (startingPose == null) setZeroToCurrent();

        desiredPsi = getFieldYaw();
        lastDesiredPsi = desiredPsi;
        sSinceDirChange = 0;

        // Bindings
        button(() -> gamepad1.right_bumper).whenBecomesTrue(() -> slowMode = !slowMode);
        button(() -> gamepad1.left_bumper).whenBecomesTrue(() -> autoHeading = !autoHeading);
        button(() -> gamepad1.y).whenBecomesTrue(() -> speedMode = !speedMode);

        button(() -> gamepad1.dpad_up).whenBecomesTrue(() -> setZeroToDeg(0));
        button(() -> gamepad1.dpad_left).whenBecomesTrue(() -> setZeroToDeg(90));
        button(() -> gamepad1.dpad_down).whenBecomesTrue(() -> setZeroToDeg(180));
        button(() -> gamepad1.dpad_right).whenBecomesTrue(() -> setZeroToDeg(270));
    }

    @Override
    public void loop() {
        BindingManager.update();
        follower.update();
        telemetryM.update();

        long nowInput = System.nanoTime();
        double dtIn = (nowInput - lastInputTsNanos) / 1e9;
        if (dtIn <= 0) dtIn = 1e-3;

        double lx = -gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        lx = deadband(lx, MOVE_DEADBAND);
        ly = deadband(ly, MOVE_DEADBAND);
        rx = deadband(rx, ROT_DEADBAND);

        lx = slew(lx, prevLx, SLEW_RATE_UNITS_PER_S, dtIn);
        ly = slew(ly, prevLy, SLEW_RATE_UNITS_PER_S, dtIn);
        prevLx = lx; prevLy = ly; lastInputTsNanos = nowInput;

        double psi = getFieldYaw();
        boolean manualRot = Math.abs(rx) > ROT_DEADBAND;

        double mag = Math.hypot(lx, ly);
        if (mag > MOVE_DEADBAND && autoHeading && !manualRot) {
            desiredPsi = Math.atan2(ly, lx);
        }

        double dPsiDes = wrap(desiredPsi - lastDesiredPsi);
        if (Math.abs(dPsiDes) > Math.toRadians(DIR_RESET_DEG)) sSinceDirChange = 0;
        sSinceDirChange += updateDistance();
        lastDesiredPsi = desiredPsi;

        if (prevManualRot && !manualRot) desiredPsi = psi;
        prevManualRot = manualRot;

        double e = wrap(desiredPsi - psi);
        double omega;
        if (manualRot) {
            omega = rx * OMEGA_MAX_RAD;
        } else if (mag <= MOVE_DEADBAND) {
            omega = KP_HOLD * e;
        } else {
            double v = mag;
            omega = clamp(v * e / L_CONV_M, -OMEGA_MAX_RAD, OMEGA_MAX_RAD);
        }

        // Field to robot transform
        double cos = Math.cos(psi), sin = Math.sin(psi);
        double fieldVx = lx, fieldVy = ly;
        double robotForward = fieldVx * cos + fieldVy * sin;
        double robotStrafe  = -fieldVx * sin + fieldVy * cos;

        double f = slowMode ? slowModeMultiplier : 1.0;

        if (!speedMode) {
            // Power control path
            follower.setTeleOpDrive(robotForward * f, robotStrafe * f, omega * f, false);
        } else {
            // Speed control path using Constants
            double vxCmd = clamp(robotForward * Constants.Speed.MAX_VEL_MPS * f,
                    -Constants.Speed.MAX_VEL_MPS, Constants.Speed.MAX_VEL_MPS);
            double vyCmd = clamp(robotStrafe  * Constants.Speed.MAX_VEL_MPS * f,
                    -Constants.Speed.MAX_VEL_MPS, Constants.Speed.MAX_VEL_MPS);
            double wzCmd = clamp(omega * f,
                    -Constants.Speed.MAX_ANG_VEL_RADPS, Constants.Speed.MAX_ANG_VEL_RADPS);

            vxCmd = slew(vxCmd, prevVxCmd, Constants.Speed.LINEAR_SLEW_MPS2, dtIn);
            vyCmd = slew(vyCmd, prevVyCmd, Constants.Speed.LINEAR_SLEW_MPS2, dtIn);
            wzCmd = slew(wzCmd, prevWzCmd, Constants.Speed.ANGULAR_SLEW_RAD2, dtIn);
            prevVxCmd = vxCmd; prevVyCmd = vyCmd; prevWzCmd = wzCmd;

            Constants.MecanumIK.WheelSpeeds ws = Constants.MecanumIK.wheelSpeedsFromChassis(vxCmd, vyCmd, wzCmd);
            Constants.MecanumIK.TicksPerSec tps = ws.toTicksPerSec();

            motors.setAllVelocityTps(tps.tFL, tps.tFR, tps.tBL, tps.tBR);
        }

        telemetryM.debug("Mode", speedMode ? "Speed" : "Power");
        telemetryM.debug("Slow Mode", slowMode);
        telemetryM.debug("AutoHeading", autoHeading);
        telemetryM.debug("Field Yaw (deg)", Math.toDegrees(psi));
        telemetryM.debug("Desired Yaw (deg)", Math.toDegrees(desiredPsi));
        telemetryM.debug("Error (deg)", Math.toDegrees(e));
        telemetryM.debug("Omega Input (rad/s)", omega);
        telemetryM.debug("sSinceDirChange(m)", sSinceDirChange);
    }

    @Override
    public void stop() {
        BindingManager.reset();
        if (speedMode && motors != null) motors.stop();
    }

    // Helpers
    private double getFieldYaw() {
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        return wrap(psiRaw + yawOffset);
    }
    private void setZeroToCurrent() {
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(0 - psiRaw);
    }
    private double updateDistance() {
        Pose p = follower.getPose();
        double dx = p.getX() - lastPose.getX();
        double dy = p.getY() - lastPose.getY();
        lastPose = p;
        return Math.hypot(dx, dy);
    }
    private void setZeroToDeg(double deg) {
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
    private static double slew(double target, double prev, double ratePerSec, double dt){
        double maxDelta = ratePerSec * dt;
        double delta = clamp(target - prev, -maxDelta, maxDelta);
        return prev + delta;
    }
}

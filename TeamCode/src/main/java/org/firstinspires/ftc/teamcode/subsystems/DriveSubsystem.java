package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotState;

public class DriveSubsystem implements Subsystem {

    // ===== Public API: modes =====
    public enum DriveMode {
        AUTO_HEADING, // translate -> face left-stick direction
        NORMAL,       // suppress auto-heading (manual-only rotation), no speed scaling
        SLOW          // suppress auto-heading + speed scaling
    }

    private final Follower follower;

    // --- Configurables (tune to taste; mirrors your TeleOp constants) ---
    public boolean poseIsMeters          = false;
    public double  moveDeadband          = 0.08;
    public double  rotDeadband           = 0.10;
    public double  omegaMaxRad           = Math.toRadians(90);
    public double  lConvMeters           = 2.5;       // heading convergence length
    public double  slewUnitsPerSec       = 3.0;       // for left stick
    public double  dirResetDeg           = 11;
    public double  slowMultiplier        = 0.40;      // SLOW mode scale
    public double  autoHeadingRearmSec   = 0.35;
    public double  manualOverrideThresh  = 0.02;      // raw rx threshold
    public double  restArmTimeSec        = 0.20;
    public double  driftTakeoverDeg      = 12.0;
    public boolean fieldOrRobotFlag      = false;     // pass-through to Pedro (false = field-centric in your setup)
    public boolean invertFieldX          = false;
    public boolean invertFieldY          = true;
    public boolean invertRotStick        = false;
    public boolean negateFieldYaw        = false;

    // --- State ---
    private Pose teleopSeedPose = null;
    private boolean robotCentric = false;

    private DriveMode defaultMode = DriveMode.AUTO_HEADING;
    private DriveMode activeMode  = DriveMode.AUTO_HEADING;

    // Yaw source (degrees). Default uses Pedro heading.
    private DoubleSupplier yawDegSupplier;

    // Auto-heading internals
    private double yawOffset = 0;         // radians
    private double desiredPsi = 0;        // radians
    private double lastDesiredPsi = 0;    // radians
    private Pose  lastPose;

    private boolean prevMoving = false;
    private boolean prevManualRot = false;
    private boolean inRest = false;
    private double  restArmingT = 0.0;    // seconds
    private double  rearmT = 0.0;         // seconds until auto-heading allowed
    private double  sSinceDirChange = 0;  // meters

    // Slew limiting
    private double prevFx = 0, prevFy = 0;
    private long   lastInputTsNanos = 0;

    public DriveSubsystem(HardwareMap hw) {
        this.follower = Constants.createFollower(hw);
        // default yaw provider from Pedro
        this.yawDegSupplier = () -> Math.toDegrees(follower.getHeading());
    }

    /** Optional: let TeleOp provide a seed before init. */
    public void setTeleopSeedPose(Pose p) { this.teleopSeedPose = p; }

    /** Set default mode used when no “hold” is active. */
    public void setDefaultMode(DriveMode mode) { this.defaultMode = mode; this.activeMode = mode; }

    /** Field-/robot-centric toggle (passes through to Pedro). */
    public void setRobotCentric(boolean rc) { this.robotCentric = rc; }
    public boolean isRobotCentric()         { return robotCentric;   }

    /** Replace yaw source (e.g., Pinpoint) in DEGREES. */
    public void setYawSupplierDegrees(DoubleSupplier degSupplier) {
        this.yawDegSupplier = degSupplier;
    }

    @Override
    public void initialize() {
        if (follower.isBusy()) {
            follower.startTeleopDrive();
        }

        Pose seed = RobotState.takeHandoffPose();
        if (seed == null) seed = (teleopSeedPose != null) ? teleopSeedPose : new Pose();

        follower.setStartingPose(seed);
        follower.update();            // warm-up tick
        follower.startTeleopDrive();  // set motor modes per Pedro config

        lastPose = follower.getPose();
        long now = System.nanoTime();
        lastInputTsNanos = now;
        prevFx = 0; prevFy = 0;

        // Initialize heading reference
        setZeroToCurrent();
        desiredPsi = getFieldYaw();
        lastDesiredPsi = desiredPsi;

        // Start in default mode
        activeMode = defaultMode;
    }

    @Override
    public void periodic() {
        follower.update();            // MUST run every loop
    }

    /** Call this on OpMode stop. */
    public void shutdown() {
        follower.startTeleopDrive();
        driveRaw(0, 0, 0, false, 1.0);
    }

    // === Main high-level API ===

    /**
     * Feed sticks + held bumpers, we’ll pick the active mode:
     * - RB held -> SLOW
     * - LB held -> NORMAL
     * - else -> defaultMode (commonly AUTO_HEADING)
     *
     * @param lx left stick X (+left)
     * @param ly left stick Y (+forward)
     * @param rx right stick X (+CCW from your convention)
     * @param rightBumperHeld slow mode when true
     * @param leftBumperHeld  normal/precision mode when true
     */
    public void driveWithModeHolds(double lx, double ly, double rx,
                                   boolean rightBumperHeld, boolean leftBumperHeld) {

        if (rightBumperHeld)      activeMode = DriveMode.SLOW;
        else if (leftBumperHeld)  activeMode = DriveMode.NORMAL;
        else                      activeMode = defaultMode;

        driveWithMode(lx, ly, rx, activeMode);
    }

    /**
     * Drive using an explicit mode (useful if you bind elsewhere).
     */
    public void driveWithMode(double lx, double ly, double rx, DriveMode mode) {
        activeMode = mode;

        // Timebase for slew + timers
        long nowInput = System.nanoTime();
        double dt = (nowInput - lastInputTsNanos) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastInputTsNanos = nowInput;

        // Map into FIELD frame (your convention: X=forward, Y=left)
        double fx = +ly;  // forward +
        double fy = +lx;  // left +
        if (invertFieldX) fx = -fx;
        if (invertFieldY) fy = -fy;

        // Rotation stick: your sign is +CCW, but Pedro wants +turnRight; we invert later in driveRaw
        double rxRaw = invertRotStick ? -rx : rx; // rxRaw used only for manual override detect
        double rxDB  = deadband(rx, rotDeadband);

        // Deadbands for move
        fx = deadband(fx, moveDeadband);
        fy = deadband(fy, moveDeadband);

        // Slew the left stick
        fx = slew(fx, prevFx, slewUnitsPerSec, dt);
        fy = slew(fy, prevFy, slewUnitsPerSec, dt);
        prevFx = fx; prevFy = fy;

        // Update timers
        rearmT = Math.max(0, rearmT - dt);

        // Current heading, magnitude, states
        double psi = getFieldYaw(); // radians
        double mag = Math.hypot(fx, fy);
        boolean isMoving  = mag > moveDeadband;
        boolean manualRot = Math.abs(rxRaw) > manualOverrideThresh;

        // Determine whether auto-heading is suppressed in this mode
        boolean suppress = (mode == DriveMode.NORMAL || mode == DriveMode.SLOW);

        // Rest detection
        boolean justStopped        =  prevMoving && !isMoving && !manualRot;
        boolean justReleasedRot    =  prevManualRot && !manualRot && !isMoving;
        boolean enteringRest       = (justStopped || justReleasedRot) || (!inRest && !isMoving && !manualRot);
        boolean leavingRest        =  inRest && (isMoving || manualRot);

        if (enteringRest) {
            desiredPsi = psi; // zero error while “resting”
            restArmingT = 0.0;
            inRest = true;
        } else if (leavingRest) {
            inRest = false;
        }
        if (inRest) restArmingT += dt;

        // Choose desired heading
        if (manualRot) {
            desiredPsi = psi;                  // track actual for clean handoff
            rearmT = autoHeadingRearmSec;      // delay before auto-heading resumes
        } else if (isMoving && !suppress && rearmT == 0) {
            desiredPsi = Math.atan2(fy, fx);   // face travel direction
        } else if (!isMoving && !manualRot) {
            double err = Math.toDegrees(Math.abs(wrap(desiredPsi - psi)));
            if (restArmingT >= restArmTimeSec && err >= driftTakeoverDeg) {
                desiredPsi = psi;              // drift kill while resting
            }
        }

        // Reset path distance if desired heading jumps
        double dPsiDes = wrap(desiredPsi - lastDesiredPsi);
        if (Math.abs(dPsiDes) > Math.toRadians(dirResetDeg)) sSinceDirChange = 0;
        sSinceDirChange += updateDistance();
        lastDesiredPsi = desiredPsi;

        // Angular command
        double e = wrap(desiredPsi - psi);
        double omega;
        if (manualRot) {
            omega = rxDB * omegaMaxRad;                    // manual
        } else if (suppress) {
            omega = 0.0;                                   // modes suppress auto-heading
        } else if (isMoving) {
            double v = mag; // proxy speed from stick
            omega = clamp(v * e / lConvMeters, -omegaMaxRad, +omegaMaxRad);
        } else {
            omega = 0.0;
        }

        // Speed multiplier for SLOW only
        double mult = (mode == DriveMode.SLOW) ? slowMultiplier : 1.0;

        // Send to Pedro (respect your existing transform):
        // follower.setTeleOpDrive(+ly * s, -lx * s, -rx * s, robotCentric)
        driveRaw(+ly, -lx, omegaToRx(omega), robotCentric, mult);

        // Bookkeeping
        prevMoving = isMoving;
        prevManualRot = manualRot;
    }

    /** Low-level pass-through (keeps your original sign conventions). */
    private void driveRaw(double ly, double lxStrafeRight, double rxTurnCCW, boolean robotCentric, double mult) {
        follower.setTeleOpDrive(
                ly * mult,
                lxStrafeRight * mult,
                -rxTurnCCW * mult,   // Pedro expects +turnRight; our rx is +CCW so negate
                robotCentric ? true : fieldOrRobotFlag
        );
    }

    // === Helpers ===

    private double getFieldYaw(){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        double psi = wrap(psiRaw + yawOffset);
        return negateFieldYaw ? -psi : psi;
    }

    /** Make current yaw the field-forward zero. Call on init or when zeroing. */
    public void setZeroToCurrent(){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(0 - psiRaw);
    }

    /** Optional absolute zero set (e.g., 90, 180, 270 helpers). */
    public void setZeroToDeg(double deg){
        double psiRaw = Math.toRadians(yawDegSupplier.getAsDouble());
        yawOffset = wrap(Math.toRadians(deg) - psiRaw);
        desiredPsi = getFieldYaw();
        lastDesiredPsi = desiredPsi;
    }

    /** Convert omega (rad/s) back to a normalized rx in [-1,1] for the Pedro call. */
    private double omegaToRx(double omega) {
        if (omegaMaxRad <= 1e-6) return 0.0;
        double rxNorm = clamp(omega / omegaMaxRad, -1.0, 1.0);
        return rxNorm;
    }

    private double updateDistance(){
        Pose p = follower.getPose();
        double dx = p.getX() - lastPose.getX();
        double dy = p.getY() - lastPose.getY();
        lastPose = p;
        // If your pose is in inches, this is inches. Auto-heading uses lConvMeters — fine as a scale factor.
        return Math.hypot(dx, dy);
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


    // Add this getter so HUD/telemetry can reflect the current mode:
    public DriveMode getActiveMode() { return activeMode; }

    public Pose2D getPose() {
        Pose p = follower.getPose();
        DistanceUnit distUnit = poseIsMeters ? DistanceUnit.METER : DistanceUnit.INCH;
        // Pedro headings are radians
        AngleUnit angUnit = AngleUnit.RADIANS;
        return new Pose2D(distUnit, p.getX(), p.getY(), angUnit, follower.getHeading());
    }


    public double getX()          { return follower.getPose().getX(); }
    public double getY()          { return follower.getPose().getY(); }
    public double getHeadingRad() { return follower.getHeading();     }

    public void setPose(double x, double y, double headingRad) {
        follower.setStartingPose(new Pose(x, y, headingRad));
        follower.startTeleopDrive();
    }
}

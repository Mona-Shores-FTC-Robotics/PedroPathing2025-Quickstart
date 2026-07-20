package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

public class FollowPathBuilder {

    private final Robot robot;
    private final Alliance alliance;

    private Pose start;
    private Pose end;


    private final List<Pose> controlPoints = new ArrayList<>();

    private boolean constantHeading = false;
    private double constantHeadingDeg = 0;
    private Double translationalConstraint = null;
    private Double headingConstraint = null;

    private boolean usePiecewiseHeading = false;
    private HeadingInterpolator piecewiseInterpolator = null;

    private double linearInterpWeight = 0.7;

    private Double fireAtT = null;
    private LauncherSubsystem launcherForCallback = null;
    private IntakeSubsystem intakeForCallback = null;

    private Double timeoutMilliSec = null;

    /** Additional segments appended via .then(). Each is built into the same PathChain. */
    private final List<SegmentSpec> additionalSegments = new ArrayList<>();

    /** Whether PedroCommands.follow holds the end pose. Default true (legacy behavior). */
    private boolean holdEnd = true;

    private static class SegmentSpec {
        Pose end;
        final List<Pose> controlPoints = new ArrayList<>();
    }

    public FollowPathBuilder(Robot robot, Alliance alliance) {
        this.robot = robot;
        this.alliance = alliance;
    }

    // ---------------------------
    // Fluent API
    // ---------------------------

    /**
     * Sets start pose from waypoint coordinates (will be mirrored for red alliance)
     */
    public FollowPathBuilder from(Pose startPose) {
        this.start = mirror(startPose);
        return this;
    }

    /**
     * Sets start pose from world coordinates (already accounts for alliance, no mirroring)
     * Use this for vision-detected poses or follower's current pose
     */
    public FollowPathBuilder fromWorldCoordinates(Pose worldStartPose) {
        this.start = worldStartPose;
        return this;
    }

    public FollowPathBuilder to(Pose endPose) {
        this.end = mirror(endPose);
        return this;
    }

    public FollowPathBuilder withControl(Pose controlPose) {
        if (additionalSegments.isEmpty()) {
            // First segment — original behavior
            this.controlPoints.add(mirror(controlPose));
        } else {
            // Route to the most recent .then() segment
            additionalSegments.get(additionalSegments.size() - 1).controlPoints.add(mirror(controlPose));
        }
        return this;
    }

    /**
     * Appends an additional segment to the path chain. The previous segment's end pose
     * becomes this segment's start; subsequent {@link #withControl(Pose)} calls accumulate
     * on this new segment. Multiple {@code .then()} calls extend the chain further.
     *
     * <p>The resulting {@link PathChain} contains all segments and is followed by Pedro
     * as one continuous motion — the robot does NOT decelerate to 0 between segments
     * within the chain.
     *
     * <p><b>Heading interpolation</b> is set ONCE per {@code build()} and applies to the
     * whole chain (Pedro PathBuilder limitation). For chained segments that need
     * different heading modes, split into separate {@code FollowPathBuilder.build()} calls.
     */
    public FollowPathBuilder then(Pose endPose) {
        if (this.end == null) {
            throw new IllegalStateException(
                "Call .to(...) before .then(...) — .then() extends a chain that begins with .to().");
        }
        SegmentSpec spec = new SegmentSpec();
        spec.end = mirror(endPose);
        additionalSegments.add(spec);
        return this;
    }

    /**
     * Whether the follower holds position at the end of the chain. Default true
     * (matches legacy behavior). Set false to yield control as soon as the path
     * completes — useful when an immediately-following command will take over.
     */
    public FollowPathBuilder withHoldEnd(boolean hold) {
        this.holdEnd = hold;
        return this;
    }

    public FollowPathBuilder withConstantHeading(double headingDeg) {
        this.constantHeading = true;
        // Mirror heading for red alliance (same logic as poseForAlliance)
        if (alliance == Alliance.RED) {
            double headingRad = Math.toRadians(headingDeg);
            double mirroredRad = Math.PI - headingRad;
            // Normalize to 0-360 range
            double mirroredDeg = Math.toDegrees(mirroredRad);
            this.constantHeadingDeg = (mirroredDeg % 360 + 360) % 360;
        } else {
            this.constantHeadingDeg = headingDeg;
        }
        return this;
    }

    public FollowPathBuilder withLinearHeadingCompletion(double w) {
        this.linearInterpWeight = w;
        return this;
    }

    public FollowPathBuilder withTranslationalConstraint(double t) {
        this.translationalConstraint = t;
        return this;
    }

    public FollowPathBuilder withHeadingConstraint(double radians) {
        this.headingConstraint = radians;
        return this;
    }

    public FollowPathBuilder withPiecewiseHeadingInterpolation(HeadingInterpolator interpolator) {
        this.usePiecewiseHeading = true;
        this.piecewiseInterpolator = interpolator;
        return this;
    }

    /**
     * Helper method to create piecewise heading interpolation with alliance mirroring.
     * First section: constant heading from t=0 to constantEndT
     * Second section: linear heading from constantEndT to t=1
     *
     * @param constantHeadingDeg Constant heading in degrees (will be mirrored for red alliance)
     * @param constantEndT T-value where constant section ends (0.0 to 1.0)
     * @param linearEndHeadingDeg End heading for linear section in degrees (will be mirrored for red alliance)
     * @return This returns itself with the updated data
     */
    public FollowPathBuilder withPiecewiseConstantThenLinear(
            double constantHeadingDeg,
            double constantEndT,
            double linearEndHeadingDeg) {

        // Mirror headings for red alliance
        double mirroredConstantHeading = mirrorHeading(constantHeadingDeg);
        double mirroredLinearEndHeading = mirrorHeading(linearEndHeadingDeg);

        HeadingInterpolator interpolator = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0.0,
                        constantEndT,
                        HeadingInterpolator.constant(Math.toRadians(mirroredConstantHeading))
                ),
                HeadingInterpolator.PiecewiseNode.linear(
                        constantEndT,
                        1.0,
                        Math.toRadians(mirroredConstantHeading),
                        Math.toRadians(mirroredLinearEndHeading)
                )
        );

        return withPiecewiseHeadingInterpolation(interpolator);
    }

    /**
     * Mirrors a heading for red alliance using the same logic as poseForAlliance
     */
    private double mirrorHeading(double headingDeg) {
        if (alliance == Alliance.RED) {
            double headingRad = Math.toRadians(headingDeg);
            double mirroredRad = Math.PI - headingRad;
            // Normalize to 0-360 range
            double mirroredDeg = Math.toDegrees(mirroredRad);
            return (mirroredDeg % 360 + 360) % 360;
        } else {
            return headingDeg;
        }
    }

    // ---------------------------
    // Build
    // ---------------------------

    public Command build(double maxPower) {
        PathBuilder builder = robot.drive.getFollower().pathBuilder();

        // ── First segment (from .from()/.to() + .withControl() calls before any .then()) ──
        addSegmentToBuilder(builder, start, end, controlPoints);

        // ── Additional segments appended via .then() ──────────────────────────────
        Pose segStart = end;
        for (SegmentSpec spec : additionalSegments) {
            addSegmentToBuilder(builder, segStart, spec.end, spec.controlPoints);
            segStart = spec.end;
        }

        // ── Heading interpolation applies to the whole chain (Pedro limitation) ──
        // For multi-segment chains, use the last segment's end for the linear endpoint.
        Pose chainEnd = additionalSegments.isEmpty()
                ? end
                : additionalSegments.get(additionalSegments.size() - 1).end;

        if (usePiecewiseHeading) {
            builder.setHeadingInterpolation(piecewiseInterpolator);
        } else if (constantHeading) {
            builder.setConstantHeadingInterpolation(Math.toRadians(constantHeadingDeg));
        } else {
            // Force shortest-direction rotation. Pedro's linear interp uses raw
            // start/end values, so if end-start lands outside [-π, π] the robot
            // turns the long way. Blue waypoints come in unnormalized
            // (Math.toRadians(deg)) while red mirrors go through
            // AngleUnit.normalizeRadians, so the two alliances can land on
            // different sides of the wrap for the same physical motion.
            double startHeading = start.getHeading();
            double endHeading = chainEnd.getHeading();
            double diff = endHeading - startHeading;
            while (diff > Math.PI)  { endHeading -= 2 * Math.PI; diff = endHeading - startHeading; }
            while (diff < -Math.PI) { endHeading += 2 * Math.PI; diff = endHeading - startHeading; }
            builder.setLinearHeadingInterpolation(
                    startHeading,
                    endHeading,
                    linearInterpWeight
            );
        }

        if (fireAtT != null && launcherForCallback != null) {
            LauncherSubsystem launcher = launcherForCallback;
            IntakeSubsystem intake = intakeForCallback;
            builder.addParametricCallback(fireAtT, () -> {
                launcher.spinUpAllLanesToLaunch();
                for (LauncherLane lane : LauncherLane.values()) {
                    launcher.queueShot(lane);
                }
                if (intake != null) {
                    intake.setGateAllowArtifacts();
                }
            });
        }

        PathChain chain = builder.build();

        // Constraints apply to path 0 (legacy behavior — applies to first leg of chain).
        // For multi-segment chains where you want a constraint on the final settling
        // pose, this would need to target chain.getPath(chain.size()-1) instead; left
        // unchanged to avoid silently changing behavior for existing single-.to() callers.
        if (timeoutMilliSec != null) {
            Path onlyPath = chain.getPath(0);
            onlyPath.setTimeoutConstraint(timeoutMilliSec);
        }
        if (headingConstraint != null) {
            Path onlyPath = chain.getPath(0);
            onlyPath.setHeadingConstraint(headingConstraint);
        }
        if (translationalConstraint != null) {
            Path onlyPath = chain.getPath(0);
            onlyPath.setTranslationalConstraint(translationalConstraint);
        }

        double clippedPower = Range.clip(maxPower, 0.0, 1.0);

        return PedroCommands.follow(robot.drive.getFollower(), chain, holdEnd, clippedPower);
    }

    /**
     * Adds one logical segment to the PathBuilder as a SINGLE Bezier whose order matches the
     * control-point count: 0 controls -> straight line, 1 control -> quadratic, 2 controls ->
     * cubic, N controls -> order-(N+1) Bezier.
     *
     * <p>This matches exactly what the Pedro visualizer renders for a line with that many
     * control points, so a path tuned by dragging control points in the visualizer drives
     * 1:1 on the robot. (Pose implements FuturePose, so Pedro's {@code BezierCurve(List<Pose>)}
     * ctor accepts our start/control/end list directly — no decomposition into sub-curves.)
     */
    private static void addSegmentToBuilder(PathBuilder builder, Pose segStart, Pose segEnd, List<Pose> controls) {
        if (controls.isEmpty()) {
            builder.addPath(new BezierLine(segStart, segEnd));
            return;
        }
        List<Pose> pts = new ArrayList<>();
        pts.add(segStart);
        pts.addAll(controls);
        pts.add(segEnd);
        builder.addPath(new BezierCurve(pts));
    }

    // ---------------------------
    // Internal: Mirror utility
    // ---------------------------

    private Pose mirror(Pose p) {
        return poseForAlliance(
                p.getX(),
                p.getY(),
                Math.toDegrees(p.getHeading()),
                alliance
        );
    }

    // This forwards to your existing method
    private static Pose poseForAlliance(double x, double y, double headingDeg, Alliance alliance) {
        return AutoField.poseForAlliance(x, y, headingDeg, alliance);
    }

    public FollowPathBuilder withTimeout(double milliseconds) {
        this.timeoutMilliSec = milliseconds;
        return this;
    }

    /**
     * Continuously rotates the robot to face a fixed field point throughout the path.
     * Heading changes along the path as the robot's position changes — use on return
     * paths where the goal is a fixed corner and the robot is moving toward it.
     */
    public FollowPathBuilder withFacingPoint(double goalX, double goalY) {
        return withPiecewiseHeadingInterpolation(
                HeadingInterpolator.facingPoint(goalX, goalY));
    }

    /**
     * Attaches a ParametricCallback that fires all launcher lanes when path T >= fireAtT.
     * Must be called before build(). fireAtT should be greater than the linearInterpWeight so
     * heading is already settled when shots queue.
     * Uses Pedro's built-in callback system — no parallel command wrapper needed.
     */
    public FollowPathBuilder withFireAtT(double t, LauncherSubsystem launcher, IntakeSubsystem intake) {
        this.fireAtT = t;
        this.launcherForCallback = launcher;
        this.intakeForCallback = intake;
        return this;
    }
}

package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ModeAwareLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.IntakeMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/**
 * Close-side "together" auto, variant 4: identical to {@link CloseTogether3Command} EXCEPT the
 * order of the second cycle is reversed — open gate #2 FIRST, THEN pick up artifact set 2.
 *
 * Sequence:
 *   1. Shoot preload          (drive to launchClose1, launch)
 *   2. Pick up set 1 AND open gate #1 — one cubic Bezier (launchClose1 -> openGate), intaking,
 *      bounded by a timeout because the robot drives into the gate.
 *   3. Shoot set 1            (return to launchClose2, launch)
 *   4. Open gate #2           (launchClose2 -> openGate2, single curve into the gate, timeout)  <-- gate BEFORE pickup
 *   5. Pick up artifact set 2 (openGate2 -> artifactsSet2, intaking)
 *   6. Shoot set 2            (return to launchClose3, launch)
 *   7. Park                   (drive to nearGate)
 *
 * Why the difference vs. variant 3: variant 3 picks up set 2 and then opens the gate
 * (artifactsSet2 -> openGate2). Variant 4 swaps that: it drives launch -> gate first, opens it,
 * then comes back down to the artifact row to collect set 2 before returning to launch. Because
 * the gate-open move now starts all the way back at the launch spot (not at the nearby pickup),
 * it's a much longer drive, so {@link Config#gatePushTimeoutMs} is larger here than in variant 3.
 * Drag the control points in Trajectories/CloseTogether4.pp to tune the new segment shapes.
 */
@Configurable
public class CloseTogether4Command {

    public static class Config {
        public double maxPathPower = .75;
        public double endTimeForLinearHeadingInterpolation = .80;
        public double secondsOpeningGate = .4;
        /** Hard cap (ms) on the gate-#2 open move. Unlike variant 3 (a short push from the
         *  adjacent pickup), variant 4 drives all the way from launchClose2 to the gate AND
         *  shoves it, so this must cover the full launch->gate drive plus the push — it's set
         *  larger than variant 3's 1500. The robot drives into the gate and never reaches the
         *  endpoint, so this timeout (not !isBusy()) is what ends the move. */
        public double gatePushTimeoutMs = 2800;
        /** Hard cap (ms) on the FUSED pickup-and-gate sweep (step 2). The whole motion ends in
         *  the gate so the endpoint is never reached and !isBusy() never fires; this bounds the
         *  entire sweep and must cover BOTH the pickup drive AND the gate shove. */
        public double pickupAndGateTimeoutMs = 3500;
    }

    public static class Waypoints {
        public double startX = 26.5;
        public double startY = 130;
        public double startHeading = 0;

        // LaunchClose1
        public double launchClose1X = 36;
        public double launchClose1Y = 107;
        public double launchClose1Heading = 130;

        // Fused pickup-and-gate (step 2) — ONE cubic Bezier from launchClose1 straight into
        // the gate, shaped by TWO control points (same as variant 3).
        public double pickupGateControl0X = 10.5;
        public double pickupGateControl0Y = 122.5;
        public double pickupGateControl1X = 38;
        public double pickupGateControl1Y = 90;

        // OpenGate #1 — endpoint of the fused move (the gate itself).
        public double openGateX = 11;
        public double openGateY = 90;
        public double openGateHeading = 270;

        // LaunchClose2
        public double launchClose2X = 36;
        public double launchClose2Y = 107;
        public double launchClose2Heading = 134;

        // Control for the gate-#1 exit (openGate -> launchClose2). Peels the robot off the gate.
        public double launchClose2Control0X = 22.7;
        public double launchClose2Control0Y = 75.2;

        // OpenGate #2 — same physical gate, but here it's opened BEFORE set-2 pickup, so it's
        // reached by driving from launchClose2 (not up from the pickup). The launch->gate start
        // and end points are IDENTICAL to the gate #1 approach (step 2), so these two controls
        // are seeded to the gate #1 control values (pickupGateControl0/1) to give the exact same
        // proven curve into the gate. Tune independently here if gate #2 needs to differ.
        public double openGate2X = 11;
        public double openGate2Y = 88;
        public double openGate2Heading = 270;
        public double gate2EntryControl0X = 10.5;
        public double gate2EntryControl0Y = 122.5;
        public double gate2EntryControl1X = 38;
        public double gate2EntryControl1Y = 88;

        // ArtifactsSet2 — picked up AFTER the gate, so it's approached from the gate (down/right).
        // set2FromGateControl0 shapes that openGate2 -> artifactsSet2 curve.
        public double artifactsSet2X = 21;
        public double artifactsSet2Y = 61.0;
        public double artifactsSet2Heading = 270;
        public double set2FromGateControl0X = 21;
        public double set2FromGateControl0Y = 80;

        // LaunchClose3
        public double launchClose3X = 36;
        public double launchClose3Y = 107;
        public double launchClose3Heading = 134;

        // Control for the set-2 shoot return (artifactsSet2 -> launchClose3). Curves the robot
        // up off the row and back to the launch spot.
        public double launchClose3FromSet2Control0X = 28;
        public double launchClose3FromSet2Control0Y = 82;

        // NearGate (park)
        public double nearGateX = 35;
        public double nearGateY = 70.4;
        public double nearGateHeading = 180.0;

        public double nearGateControl0X = 41;
        public double nearGateControl0Y = 85;
    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private CloseTogether4Command() {}

    /**
     * Gets the default start pose from waypoints (before alliance mirroring).
     * This is the fallback when vision initialization is not available.
     * @return Default start pose
     */
    public static Pose getDefaultStartPose() {
        return start();
    }

    /**
     * Creates the autonomous command sequence.
     * @param robot Robot instance with all subsystems
     * @param alliance Current alliance (BLUE or RED)
     * @return Complete autonomous command
     */
    public static Command create(Robot robot, Alliance alliance) {
        return create(robot, alliance, null);
    }

    /**
     * Creates the autonomous command sequence with optional start pose override.
     * @param robot Robot instance with all subsystems
     * @param alliance Current alliance (BLUE or RED)
     * @param startOverride Vision-detected start pose (or null to use waypoints)
     * @return Complete autonomous command
     */
    public static Command create(Robot robot, Alliance alliance, Pose startOverride) {
        // Build first path: start -> launch position
        FollowPathBuilder firstPathBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            // Vision detected: use follower's current pose (world coordinates, no mirroring)
            firstPathBuilder.fromWorldCoordinates(robot.drive.getFollower().getPose());
        } else {
            // No vision: use waypoint start pose (will be mirrored for red alliance)
            firstPathBuilder.from(start());
        }

        return Groups.sequential(
                // Reset timer when auto actually starts (not when command is created)
                ConditionalFinalLaunchCommand.createTimerReset(),

                // Pre-spin the flywheels to speed while stationary BEFORE the first drive.
                PresetRangeSpinCommand.create(
                        robot.launcher, LauncherRange.SHORT_AUTO, true,
                        robot.drive, robot.lighting, null),

                // 1. Shoot preload
                Groups.deadline(
                        firstPathBuilder
                                .to(launchClose1())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        robot.intake.setIntakeModeCmd(IntakeMode.PASSIVE_REVERSE),
                        PresetRangeSpinCommand.create(
                                robot.launcher, LauncherRange.SHORT_AUTO, true,
                                robot.drive, robot.lighting, null) // Hold SHORT RPM for the whole auto
                ),
                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // 2. Pick up artifact set 1 AND open gate #1 in ONE continuous curve (same as
                //    variant 3). Intake runs the whole sweep; bounded by a timeout because the
                //    move ends by driving into the gate.
                Groups.race(
                        Groups.deadline(
                                new FollowPathBuilder(robot, alliance)
                                        .from(launchClose1())
                                        .to(openGate())
                                        .withControl(pickupGateControl0())
                                        .withControl(pickupGateControl1())
                                        .withConstantHeading(270)
                                        .build(config.maxPathPower),
                                robot.intake.autoSmartIntakeCmd()
                        ),
                        Commands.waitMs(config.pickupAndGateTimeoutMs)
                ),
                Commands.waitMs(config.secondsOpeningGate * 1000.0), // dwell so the gate opens

                // 3. Shoot set 1 (return to launch; constant 270 then turn to launch heading).
                //    Control point peels the robot off the gate cleanly before swinging back.
                new FollowPathBuilder(robot, alliance)
                        .from(openGate())
                        .to(launchClose2())
                        .withControl(launchClose2Control0())
                        .withPiecewiseConstantThenLinear(270, 0.2, waypoints.launchClose2Heading)
                        .build(config.maxPathPower),
                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // 4. Open gate #2 FIRST — BEFORE picking up set 2, drive launch -> gate using the
                //    SAME two-control curve as the gate #1 approach (step 2). The start/end points
                //    are identical to gate #1, so this reuses that proven shape into the gate.
                //    Bounded by a timeout (larger than variant 3's, since this includes the full
                //    drive) because the move ends by driving into the gate and never reaches the
                //    endpoint.
                Groups.race(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose2())
                                .to(openGate2())
                                .withControl(gate2EntryControl0())
                                .withControl(gate2EntryControl1())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        Commands.waitMs(config.gatePushTimeoutMs)
                ),
                Commands.waitMs(config.secondsOpeningGate * 1000.0), // dwell so the gate opens

                // 5. Pick up artifact set 2 — now AFTER the gate, so the robot comes back down
                //    from the gate to the artifact row, intaking.
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(openGate2())
                                .to(artifactsSet2())
                                .withControl(set2FromGateControl0())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),

                // 6. Shoot set 2 — return from the artifact row to launch (constant 270 then turn
                //    to launch heading).
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet2())
                        .to(launchClose3())
                        .withControl(launchClose3FromSet2Control0())
                        .withPiecewiseConstantThenLinear(270, 0.2, waypoints.launchClose3Heading)
                        .build(config.maxPathPower),
                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // 7. Park
                new FollowPathBuilder(robot, alliance)
                        .from(launchClose3())
                        .to(nearGate())
                        .withControl(nearGateControl0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower)
        );
    }

    private static Pose start() {
        return new Pose(waypoints.startX, waypoints.startY, Math.toRadians(waypoints.startHeading));
    }

    private static Pose launchClose1() {
        return new Pose(waypoints.launchClose1X, waypoints.launchClose1Y, Math.toRadians(waypoints.launchClose1Heading));
    }

    private static Pose pickupGateControl0() {
        return new Pose(waypoints.pickupGateControl0X, waypoints.pickupGateControl0Y, 0);
    }

    private static Pose pickupGateControl1() {
        return new Pose(waypoints.pickupGateControl1X, waypoints.pickupGateControl1Y, 0);
    }

    private static Pose openGate() {
        return new Pose(waypoints.openGateX, waypoints.openGateY, Math.toRadians(waypoints.openGateHeading));
    }

    private static Pose launchClose2() {
        return new Pose(waypoints.launchClose2X, waypoints.launchClose2Y, Math.toRadians(waypoints.launchClose2Heading));
    }

    private static Pose launchClose2Control0() {
        return new Pose(waypoints.launchClose2Control0X, waypoints.launchClose2Control0Y, 0);
    }

    private static Pose openGate2() {
        return new Pose(waypoints.openGate2X, waypoints.openGate2Y, Math.toRadians(waypoints.openGate2Heading));
    }

    private static Pose gate2EntryControl0() {
        return new Pose(waypoints.gate2EntryControl0X, waypoints.gate2EntryControl0Y, 0);
    }

    private static Pose gate2EntryControl1() {
        return new Pose(waypoints.gate2EntryControl1X, waypoints.gate2EntryControl1Y, 0);
    }

    private static Pose artifactsSet2() {
        return new Pose(waypoints.artifactsSet2X, waypoints.artifactsSet2Y, Math.toRadians(waypoints.artifactsSet2Heading));
    }

    private static Pose set2FromGateControl0() {
        return new Pose(waypoints.set2FromGateControl0X, waypoints.set2FromGateControl0Y, 0);
    }

    private static Pose launchClose3() {
        return new Pose(waypoints.launchClose3X, waypoints.launchClose3Y, Math.toRadians(waypoints.launchClose3Heading));
    }

    private static Pose launchClose3FromSet2Control0() {
        return new Pose(waypoints.launchClose3FromSet2Control0X, waypoints.launchClose3FromSet2Control0Y, 0);
    }

    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

    private static Pose nearGateControl0() {
        return new Pose(waypoints.nearGateControl0X, waypoints.nearGateControl0Y, 0);
    }
}

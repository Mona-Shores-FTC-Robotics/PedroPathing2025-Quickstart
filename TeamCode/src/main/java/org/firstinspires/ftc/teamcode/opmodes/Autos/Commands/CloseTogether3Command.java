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
 * Close-side "together" auto, variant 3: identical to {@link CloseTogether2Command} except the
 * first pickup and first gate push are fused into ONE continuous motion.
 *
 * Sequence:
 *   1. Shoot preload          (drive to launchClose1, launch)
 *   2. Pick up set 1 AND open gate #1 — ONE single cubic Bezier (launchClose1 -> openGate,
 *      shaped by two control points), no intermediate waypoint and no stop, intaking the
 *      whole way, bounded by a timeout because the robot drives into the gate.
 *   3. Shoot set 1            (return to launchClose2, launch)
 *   4. Pick up artifact set 2
 *   5. Open gate #2           (stage off the gate, then push, bounded by timeout)
 *   6. Shoot set 2            (return to launchClose3, launch)
 *   7. Park                   (drive to nearGate)
 *
 * Why the difference: in variant 2 the robot decelerated to 0 at artifactsSet1 before pushing
 * the gate (two separate moves). Here that whole pickup-into-gate is a single Bezier curve with
 * two control points — no intermediate waypoint to settle at, so the robot sweeps down the
 * artifact row and curves straight into the gate continuously. Because it's one true cubic, it
 * matches what the Pedro visualizer renders for a line with two control points: drag the two
 * controls in Trajectories/CloseTogether3.pp and the robot drives exactly what you see.
 */
@Configurable
public class CloseTogether3Command {

    public static class Config {
        public double maxPathPower = .80;
        public double endTimeForLinearHeadingInterpolation = .80;
        public double secondsOpeningGate = .4;
        /** Hard cap (ms) on each gate-push move. The robot drives into the gate and may
         *  never reach the path endpoint, so Follow's !isBusy() completion never fires —
         *  this timeout ends the push so the auto can't hang there. Tune alongside the
         *  openGate waypoints: enough time to reach the gate and shove it open. */
        public double gatePushTimeoutMs = 1500;
        /** Hard cap (ms) on the FUSED pickup-and-gate sweep (step 2). Unlike variant 2's
         *  separate pickup (completes naturally) + gate push (timeout), here the whole
         *  motion ends in the gate so the chain endpoint is never reached and !isBusy()
         *  never fires. This timeout bounds the entire sweep, so it must cover BOTH the
         *  pickup drive AND the gate shove — set it longer than gatePushTimeoutMs. */
        public double pickupAndGateTimeoutMs = 3500;
    }

    public static class Waypoints {
        public double startX = 26.5;
        public double startY = 130;
        public double startHeading = 0;

        // LaunchClose1
        public double launchClose1X = 36;
        public double launchClose1Y = 107;
        public double launchClose1Heading = 134;

        // Fused pickup-and-gate (step 2) — ONE cubic Bezier from launchClose1 straight into
        // the gate, shaped by TWO control points so the robot sweeps down the artifact row
        // (hugging the wall line near x=21) and curves into the gate with NO intermediate
        // waypoint, so there is no settle/stop and no curvature kink. These two controls are
        // exactly what you drag in the Pedro visualizer (Trajectories/CloseTogether3.pp) — the
        // robot drives the same cubic 1:1.
        //   control0 — high/left: bows the curve out to the wall at the top of the row
        //   control1 — right/low: shapes the back half before it cuts left into the gate
        // (Values tuned by hand in the Pedro visualizer; keep this in sync with CloseTogether3.pp.)
        public double pickupGateControl0X = 10.5;
        public double pickupGateControl0Y = 122.5;
        public double pickupGateControl1X = 38;
        public double pickupGateControl1Y = 73.5;

        // OpenGate #1 — endpoint of the fused move (the gate itself).
        public double openGateX = 11;
        public double openGateY = 78;
        public double openGateHeading = 270;

        // LaunchClose2
        public double launchClose2X = 36;
        public double launchClose2Y = 107;
        public double launchClose2Heading = 134;

        // Control for the return-from-gate move (openGate -> launchClose2). Sits just off the
        // gate so the robot peels away cleanly before swinging back to the launch spot rather
        // than dragging straight off the wall.
        public double launchClose2Control0X = 22.7;
        public double launchClose2Control0Y = 75.2;

        // ArtifactsSet2
        public double artifactsSet2X = 21;
        public double artifactsSet2Y = 61.0;
        public double artifactsSet2Heading = 270;

        public double artifactsSet2Control0X = 21;
        public double artifactsSet2Control0Y = 100;

        // OpenGate #2 — same physical gate, approached from set 2 (lower Y). Control sits
        // midway between artifactsSet2 (24, 61) and the gate, on the line. Tune independently.
        public double openGate2X = 11;
        public double openGate2Y = 78;
        public double openGate2Heading = 270;
        public double openGate2ControlX = 17;
        public double openGate2ControlY = 78;

        // LaunchClose3
        public double launchClose3X = 36;
        public double launchClose3Y = 107;
        public double launchClose3Heading = 134;

        // Control for the gate-#2 exit (openGate2 -> launchClose3). Same role/value as
        // launchClose2Control0 — peels the robot off the gate cleanly. (openGate2/launchClose3
        // share coordinates with openGate/launchClose2, so this mirrors the gate-#1 exit.)
        public double launchClose3Control0X = 22.7;
        public double launchClose3Control0Y = 75.2;

        // NearGate (park)
        public double nearGateX = 35;
        public double nearGateY = 70.4;
        public double nearGateHeading = 180.0;

        public double nearGateControl0X = 41;
        public double nearGateControl0Y = 85;
    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private CloseTogether3Command() {}

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
                // Three flywheels can't pull their startup inrush while the drive also pulls
                // hard off the line, so spinning up during the first path browns out the
                // launcher and it never starts. Get them up first, then move (the launcher
                // periodic holds the RPM target for the rest of the routine).
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

                // 2. Pick up artifact set 1 AND open gate #1 in ONE continuous curve. A single
                //    cubic Bezier from launchClose1 straight into openGate, shaped by TWO control
                //    points so it sweeps down the artifact row and curves into the gate with NO
                //    intermediate waypoint — so there's no settle/stop and no curvature kink. It's
                //    a true cubic (matches what the Pedro visualizer renders for a 2-control line),
                //    so you can drag the controls in the visualizer and the robot drives what you
                //    see. See Trajectories/CloseTogether3.pp.
                //
                //    Intake runs the whole sweep (deadline). The move is bounded by a timeout
                //    (race) because it ends by driving into the gate, so the path endpoint is
                //    never reached and the follow's !isBusy() never fires.
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

                // 4. Pick up artifact set 2
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose2())
                                .to(artifactsSet2())
                                .withControl(artifactsSet2Control0())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),

                // 5. Open gate #2 — single curve from the pickup straight into the gate, mirroring
                //    gate #1's entry (no stage-in). The control point shapes the approach so the
                //    robot curves into the gate instead of backing into it. Bounded by a timeout
                //    because the move ends by driving into the gate, so the path endpoint is never
                //    reached and the follow's !isBusy() never fires.
                Groups.race(
                        new FollowPathBuilder(robot, alliance)
                                .from(artifactsSet2())
                                .to(openGate2())
                                .withControl(openGate2Control())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        Commands.waitMs(config.gatePushTimeoutMs)
                ),
                Commands.waitMs(config.secondsOpeningGate * 1000.0), // dwell so the gate opens

                // 6. Shoot set 2 — single curve off the gate back to launch, mirroring gate #1's
                //    exit. The control point peels the robot off the gate cleanly (no need for
                //    the gate2Stage back-off); constant 270 then turn to launch heading.
                new FollowPathBuilder(robot, alliance)
                        .from(openGate2())
                        .to(launchClose3())
                        .withControl(launchClose3Control0())
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

    private static Pose artifactsSet2() {
        return new Pose(waypoints.artifactsSet2X, waypoints.artifactsSet2Y, Math.toRadians(waypoints.artifactsSet2Heading));
    }

    private static Pose artifactsSet2Control0() {
        return new Pose(waypoints.artifactsSet2Control0X, waypoints.artifactsSet2Control0Y, 0);
    }

    private static Pose openGate2() {
        return new Pose(waypoints.openGate2X, waypoints.openGate2Y, Math.toRadians(waypoints.openGate2Heading));
    }

    private static Pose openGate2Control() {
        return new Pose(waypoints.openGate2ControlX, waypoints.openGate2ControlY, 0);
    }

    private static Pose launchClose3() {
        return new Pose(waypoints.launchClose3X, waypoints.launchClose3Y, Math.toRadians(waypoints.launchClose3Heading));
    }

    private static Pose launchClose3Control0() {
        return new Pose(waypoints.launchClose3Control0X, waypoints.launchClose3Control0Y, 0);
    }

    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

    private static Pose nearGateControl0() {
        return new Pose(waypoints.nearGateControl0X, waypoints.nearGateControl0Y, 0);
    }
}

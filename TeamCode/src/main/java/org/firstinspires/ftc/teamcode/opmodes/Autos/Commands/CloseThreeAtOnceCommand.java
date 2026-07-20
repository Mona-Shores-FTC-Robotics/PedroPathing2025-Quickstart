package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ModeAwareLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.IntakeMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/**
 * Generated autonomous command from Pedro Pathing .pp file
 * Usage in OpMode:
 *   Command auto = LocalizeCommand.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 * 
 */
public class CloseThreeAtOnceCommand {

    public static class Config {
        public double maxPathPower = .8;
        // Was 1.0 — the final launch (launchClose4) drove in faster than the other launches and
        // overshot heading where they settled (#7 hErr 8.4° / angVel 29 vs #1/#3/#5 settled at
        // 0.8 power). Matched to maxPathPower so it arrives at the same speed and settles too.
        public double lastPathsMaxPower = .8;
        public double endTimeForLinearHeadingInterpolation = .8;
        public double autoDurationSeconds = 30.0;
        public double minTimeForFinalLaunchSeconds = 6.8;
        /** Heading tolerance (degrees) Pedro must reach before considering a
         *  launch-position path complete. Tighter = waits longer for the
         *  heading PID to settle before the launcher fires. */
        public double launchHeadingConstraintDeg = 1.0;
    }

    public static class Waypoints {
        public double startX = 26.5;
        public double startY = 130;
        public double startHeading = 0;

        // LaunchClose1
        public double launchClose1X = 36;
        public double launchClose1Y = 107.0;
        public double launchClose1Heading = 134;

        // ArtifactsSet1
        public double artifactsSet1X = 21;
        public double artifactsSet1Y = 83.8;
        public double artifactsSet1Heading = 270.0;


        // Control point for segment: ArtifactsSet3
        public double artifactsSet1Control0X = 20;
        public double artifactsSet1Control0Y = 120;

        // LaunchClose2
        public double launchClose2X = 36;
        public double launchClose2Y = 107.0;
        public double launchClose2Heading = 134.0;

        // ArtifactsSet2
        public double artifactsSet2X = 21;
        public double artifactsSet2Y = 61.0;
        public double artifactsSet2Heading = 270;

        // Control point for segment: ArtifactsSet2
        public double artifactsSet2Control0X = 21;
        public double artifactsSet2Control0Y = 100;

        // LaunchClose3
        public double launchClose3X = 36;
        public double launchClose3Y = 107.0;
        public double launchClose3Heading = 134.0;

        // Control point for segment: LaunchClose3
        public double launchClose3Control0X = 50.5;
        public double launchClose3Control0Y = 72;

        // ArtifactsSet3
        public double artifactsSet3X = 21;
        public double artifactsSet3Y = 35.5;
        public double artifactsSet3Heading = 260;

        // Control point for segment: ArtifactsSet3
        public double artifactsSet3Control0X = 23;
        public double artifactsSet3Control0Y = 100;

        // LaunchClose4
        public double launchClose4X = 36;
        public double launchClose4Y = 107.0;
        public double launchClose4Heading = 134.0;

        // Control point for segment: LaunchOffLine
        public double launchClose4Control0X = 44.5;
        public double launchClose4Control0Y = 73.5;

        // NearGate
        public double nearGateX = 35;
        public double nearGateY = 70.4;
        public double nearGateHeading = 180.0;

        // Control point for segment: NearGate
        public double nearGateControl0X = 41;
        public double nearGateControl0Y = 85;
    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private CloseThreeAtOnceCommand() {}

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

        Command mainSequence = Groups.sequential(
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

                // Launch Preloads
                Groups.deadline(
                        firstPathBuilder
                            .to(launchClose1())
                            .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                            .withHeadingConstraint(Math.toRadians(config.launchHeadingConstraintDeg))
                            .build(config.maxPathPower),
                        robot.intake.setIntakeModeCmd(IntakeMode.PASSIVE_REVERSE),
                        PresetRangeSpinCommand.create(
                                robot.launcher, LauncherRange.SHORT_AUTO, true,
                                robot.drive, robot.lighting, null) // Spin up to SHORT RPM for the whole auto
                ),

                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // Pickup Artifact Set 1
                Groups.deadline(
                    new FollowPathBuilder(robot, alliance)
                        .from(launchClose1())
                        .to(artifactsSet1())
                        .withControl(artifactsSet1Control0())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),
                        Groups.sequential(
                                robot.intake.autoSmartIntakeCmd()
                        )
                ),

                // Return and Launch Set 1
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet1())
                        .to( launchClose2())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .withHeadingConstraint(Math.toRadians(config.launchHeadingConstraintDeg))
                        .build(config.maxPathPower),

                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                Groups.deadline(
                    // Pickup Artifact Set 2
                    new FollowPathBuilder(robot, alliance)
                            .from(launchClose2())
                            .to(artifactsSet2())
                            .withControl(artifactsSet2Control0())
                            .withConstantHeading(270)
                            .build(config.maxPathPower),
                    Groups.sequential(
                        robot.intake.autoSmartIntakeCmd()
                    )
                ),


                // Return and Launch Set 2
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet2())
                        .to(launchClose3())
                        .withControl(launchClose3Control0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .withHeadingConstraint(Math.toRadians(config.launchHeadingConstraintDeg))
                        .build(config.maxPathPower),

                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // Pickup Artifact Set 3
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose3())
                                .to(artifactsSet3())
                                .withControl(artifactsSet3Control0())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        Groups.sequential(
                                robot.intake.autoSmartIntakeCmd()
                        )
                ),

                // Conditionally return and launch if time permits, otherwise go straight to park
                ConditionalFinalLaunchCommand.create(
                        config.autoDurationSeconds,
                        config.minTimeForFinalLaunchSeconds,
                        // If enough time: return to launch, shoot, then park
                        Groups.sequential(
                                new FollowPathBuilder(robot, alliance)
                                        .from(artifactsSet3())
                                        .to(launchClose4())
                                        .withControl(launchClose4Control0())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .withHeadingConstraint(Math.toRadians(config.launchHeadingConstraintDeg))
                                        .build(config.lastPathsMaxPower),

                                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                                Groups.deadline(
                                        new FollowPathBuilder(robot, alliance)
                                            .from(launchClose4())
                                            .to(nearGate())
                                            .withControl(nearGateControl0())
                                            .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                            .build(config.lastPathsMaxPower),
                                        Groups.sequential(
                                                robot.intake.autoSmartIntakeCmd()
                                        )
                                )
                        ),
                        // If not enough time: go straight to park
                        Groups.sequential(
                                new FollowPathBuilder(robot, alliance)
                                        .from(artifactsSet3())
                                        .to(nearGate())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower)
                        )
                )
        );

        return
                mainSequence;
    }

    private static Pose start() {
        return new Pose(waypoints.startX, waypoints.startY, Math.toRadians(waypoints.startHeading));
    }

    private static Pose launchClose1() {
        return new Pose(waypoints.launchClose1X, waypoints.launchClose1Y, Math.toRadians(waypoints.launchClose1Heading));
    }

    private static Pose artifactsSet1() {
        return new Pose(waypoints.artifactsSet1X, waypoints.artifactsSet1Y, Math.toRadians(waypoints.artifactsSet1Heading));
    }

    private static Pose artifactsSet1Control0() {
        return new Pose(waypoints.artifactsSet1Control0X, waypoints.artifactsSet1Control0Y, 0);
    }

    private static Pose launchClose2() {
        return new Pose(waypoints.launchClose2X, waypoints.launchClose2Y, Math.toRadians(waypoints.launchClose2Heading));
    }

    private static Pose artifactsSet2() {
        return new Pose(waypoints.artifactsSet2X, waypoints.artifactsSet2Y, Math.toRadians(waypoints.artifactsSet2Heading));
    }

    private static Pose artifactsSet2Control0() {
        return new Pose(waypoints.artifactsSet2Control0X, waypoints.artifactsSet2Control0Y, 0);
    }

//    private static Pose artifactsSet2Control1() {
//        return new Pose(waypoints.artifactsSet2Control1X, waypoints.artifactsSet2Control1Y, 0);
//    }

    private static Pose launchClose3() {
        return new Pose(waypoints.launchClose3X, waypoints.launchClose3Y, Math.toRadians(waypoints.launchClose3Heading));
    }

    private static Pose launchClose3Control0() {
        return new Pose(waypoints.launchClose3Control0X, waypoints.launchClose3Control0Y, 0);
    }

    private static Pose artifactsSet3() {
        return new Pose(waypoints.artifactsSet3X, waypoints.artifactsSet3Y, Math.toRadians(waypoints.artifactsSet3Heading));
    }

    private static Pose artifactsSet3Control0() {
        return new Pose(waypoints.artifactsSet3Control0X, waypoints.artifactsSet3Control0Y, 0);
    }

//    private static Pose artifactsSet3Control1() {
//        return new Pose(waypoints.artifactsSet3Control1X, waypoints.artifactsSet3Control1Y, 0);
//    }

    private static Pose launchClose4() {
        return new Pose(waypoints.launchClose4X , waypoints.launchClose4Y , Math.toRadians(waypoints.launchClose4Heading));
    }

    private static Pose launchClose4Control0() {
        return new Pose(waypoints.launchClose4Control0X , waypoints.launchClose4Control0Y , 0);
    }

    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

    private static Pose nearGateControl0() {
        return new Pose(waypoints.nearGateControl0X, waypoints.nearGateControl0Y, 0);
    }

}

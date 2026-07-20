package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.FarThreeAtOnceCommand;

/**
 * Autonomous: drive from far start, collect and score three samples at LAUNCH_FAR.
 */
@Autonomous(name = "Far Three At Once", group = "Auto")
public class DecodeAutonomousFarThreeAtOnce extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return FarThreeAtOnceCommand.waypoints.startX; }

    @Override
    protected double getStartY() { return FarThreeAtOnceCommand.waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return FarThreeAtOnceCommand.waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return FarThreeAtOnceCommand.create(robot, activeAlliance, startPoseOverride);
    }
}

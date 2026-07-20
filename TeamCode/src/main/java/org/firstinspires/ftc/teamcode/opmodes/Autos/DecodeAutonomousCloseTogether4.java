package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.CloseTogether4Command;

/**
 * Autonomous: close-side "together" variant 4 — same as variant 3 but the second cycle opens
 * gate #2 BEFORE picking up artifact set 2. See {@link CloseTogether4Command} for the full
 * sequence.
 */
@Autonomous(name = "Close Together 4", group = "Auto")
public class DecodeAutonomousCloseTogether4 extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return CloseTogether4Command.waypoints.startX; }

    @Override
    protected double getStartY() { return CloseTogether4Command.waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return CloseTogether4Command.waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return CloseTogether4Command.create(robot, activeAlliance, startPoseOverride);
    }
}

package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.CloseTogether3Command;

/**
 * Autonomous: close-side "together" variant 3 — same as variant 2 but the first pickup and
 * first gate push are fused into one continuous motion (no stop at the artifact line). See
 * {@link CloseTogether3Command} for the full sequence.
 */
@Autonomous(name = "Close Together 3", group = "Auto")
public class DecodeAutonomousCloseTogether3 extends BaseAutonomousOpMode {

    @Override
    protected double getStartX() { return CloseTogether3Command.waypoints.startX; }

    @Override
    protected double getStartY() { return CloseTogether3Command.waypoints.startY; }

    @Override
    protected double getStartHeadingDeg() { return CloseTogether3Command.waypoints.startHeading; }

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return CloseTogether3Command.create(robot, activeAlliance, startPoseOverride);
    }
}

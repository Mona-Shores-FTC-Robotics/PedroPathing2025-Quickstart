package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PsiKitAdapter;
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;

@TeleOp(name = "TeleOp")
public class Teleop extends NextFTCOpMode {

    private Robot robot;
    private TeleopBindings bindings;
    private TelemetryPublisher pub;
    private PsiKitAdapter logger;

    @Override
    public void onInit() {
        // Hardware + app objects
        robot    = new Robot(hardwareMap);
        GamepadEx driver = new GamepadEx(() -> gamepad1);
        GamepadEx operator = new GamepadEx(() -> gamepad2);
        bindings = new TeleopBindings(driver , operator , robot);

        logger = new PsiKitAdapter();
        logger.startSession();
        pub = new TelemetryPublisher(logger);

        robot.drive.setPose(0.0, 0.0, 0.0);

        // Now that subsystems exist, add them via SubsystemComponent so their
        // initialize() and periodic() are called automatically each loop.
        addComponents(new SubsystemComponent(robot.flywheel, robot.drive));
    }

    @Override
    public void onUpdate() {
        // If your drive isn’t command-based yet, keep direct control for now:
        robot.drive.drive(gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x);

        // Commands, subsystem periodic, and bindings are handled by components
        // automatically each onUpdate() — no explicit scheduler.run() needed.

        // Telemetry
        double currentRPM = robot.flywheel.getRpm();

        pub.publishDrive(
                robot.drive,
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                robot.drive.isSlowMode());

        pub.publishFlywheel(
                robot.flywheel.getTargetRpm(),
                currentRPM,
                robot.flywheel.getLastPower(),
                robot.flywheel.getTargetRpm() - currentRPM);

        bindings.publishHelp(telemetry);

        telemetry.addData("pose", "(%.1f, %.1f) h=%.1f°",
                robot.drive.getPose().x,
                robot.drive.getPose().y,
                Math.toDegrees(robot.drive.getPose().headingRad));
        telemetry.addData("flywheel", "%.0f / %.0f rpm",
                currentRPM,
                robot.flywheel.getTargetRpm());
        telemetry.update();
    }

    @Override
    public void onStop() {
        // Components will stop commands; still safe to stop hardware explicitly
        robot.flywheel.stop();
        robot.drive.shutdown();   // call our explicit cleanup
        bindings.reset();
        logger.stopSession();
    }

}

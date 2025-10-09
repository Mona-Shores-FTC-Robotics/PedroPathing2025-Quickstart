package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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
        robot = new Robot(hardwareMap);

        GamepadEx driver   = new GamepadEx(() -> gamepad1);
        GamepadEx operator = new GamepadEx(() -> gamepad2);
        bindings = new TeleopBindings(driver, operator, robot);

        logger = new PsiKitAdapter();
        logger.startSession();
        pub = new TelemetryPublisher(logger);

        // Configure drive BEFORE components so initialize() sees these settings.
        robot.drive.setDefaultMode(DriveSubsystem.DriveMode.NORMAL); // default mode
        robot.drive.setRobotCentric(false); // field-centric
        robot.drive.setPose(0.0, 0.0, 0.0);

        // Register components so NextFTC handles initialize()/periodic()/stop()
        addComponents(
                new SubsystemComponent(robot.flywheel, robot.drive));
        // NOTE: Do NOT call robot.drive.initialize() here; SubsystemComponent will handle it.
    }

    @Override
    public void onUpdate() {
        // Drive using “mode holds”: RB=SLOW, LB=NORMAL, else default (AUTO_HEADING)
        double lx = gamepad1.left_stick_x;     // +left
        double ly = gamepad1.left_stick_y;    // +forward
        double rx = -gamepad1.right_stick_x;   // +CCW

        boolean rb = gamepad1.right_bumper;    // SLOW (scaled)
        boolean lb = gamepad1.left_bumper;     // NORMAL (no scaling)

        robot.drive.driveWithModeHolds(lx, -ly, rx, rb, lb);

        // Telemetry
        double currentRPM = robot.flywheel.getRpm();

        pub.publishDrive(
                robot.drive,
                lx, ly, -rx,              // publisher kept as before; adjust signs if needed
                (rb)                      // “slow mode active” for the HUD (matches our holds)
        );

        pub.publishFlywheel(
                robot.flywheel.getTargetRpm(),
                currentRPM,
                robot.flywheel.getLastPower(),
                robot.flywheel.getTargetRpm() - currentRPM
        );

        bindings.publishHelp(telemetry);

        Pose2D pose = robot.drive.getPose();

        double x    = pose.getX(DistanceUnit.INCH);                          // in pose.getUnit()
        double y    = pose.getY(DistanceUnit.INCH);                          // in pose.getUnit()
        double hDeg = pose.getHeading(AngleUnit.DEGREES);   // convert from radians if needed

        telemetry.addData("pose", "(%.1f, %.1f) %s  h=%.1f°",
                x, y, "DEGREES", hDeg);

        telemetry.addData("flywheel", "%.0f / %.0f rpm",
                currentRPM, robot.flywheel.getTargetRpm());
        telemetry.update();
    }

    @Override
    public void onStop() {
        // SubsystemComponent will call stop; still fine to be explicit for hardware safety
        robot.flywheel.stop();
        robot.drive.shutdown();
        bindings.reset();
        logger.stopSession();
    }
}

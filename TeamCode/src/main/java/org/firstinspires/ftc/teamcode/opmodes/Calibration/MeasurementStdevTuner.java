package org.firstinspires.ftc.teamcode.opmodes.Calibration;

import static org.firstinspires.ftc.teamcode.util.RobotState.packet;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ControlHubIdentifierUtil;
import org.firstinspires.ftc.teamcode.util.WelfordVariance;

import java.util.List;

/**
 * Tune the AprilTag measurement standard deviations that feed the Kalman R
 * matrix in {@code Constants.createFollower(...)}.
 *
 * <p>Adapted from BeepBot99/CodeBloodedDecodeV3's {@code MeasurementStdevTuner},
 * but instead of standing up an isolated {@code AprilTagProcessor} pipeline this
 * samples {@link VisionSubsystemLimelight#getRobotPoseFromTagPedro()} — the
 * exact MT2-derived pose that {@code DriveSubsystem.maybeFeedVisionMeasurement()}
 * hands to {@code FusionLocalizer.addMeasurement(...)}. Tuning what the filter
 * actually consumes is the whole point.
 *
 * <p>Procedure:
 * <ol>
 *   <li>Place the robot at a known pose where AprilTag 20 or 24 is visible.</li>
 *   <li>In FTC Dashboard, set {@code Truth.actualX/Y/HeadingDeg} to that pose.</li>
 *   <li>Press PLAY. Each new Limelight snapshot contributes one sample.</li>
 *   <li>Don't move the robot — the goal is to measure measurement noise, not
 *       motion. To resample from a different pose, hit gamepad1 cross to reset,
 *       move the robot, update Truth, and continue.</li>
 *   <li>Read the running stdev values; drop into
 *       {@code Constants.createFollower(...)} as the {@code Pose(xStd, yStd, headingStd)}.</li>
 * </ol>
 *
 * <p>Heading residual is wrapped to [-180, 180] so a measurement at 179° vs.
 * truth at -179° produces a 2° error, not 358°.
 */
@Disabled
@TeleOp(name = "Measurement Stdev Tuner", group = "Calibration")
@Configurable
public class MeasurementStdevTuner extends OpMode {

    /** Known robot pose in Pedro frame (inches, degrees). Edit via Dashboard Config. */
    @Configurable
    public static class Truth {
        public static double actualXPedro = 0.0;
        public static double actualYPedro = 0.0;
        public static double actualHeadingDeg = 0.0;
    }

    private Robot robot;
    private List<LynxModule> hubs;
    private FtcDashboard dashboard;

    private final WelfordVariance xVar = new WelfordVariance();
    private final WelfordVariance yVar = new WelfordVariance();
    private final WelfordVariance headingVar = new WelfordVariance();

    private long lastSampledNs = Long.MIN_VALUE;
    private boolean prevCross = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(h -> h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        dashboard = FtcDashboard.getInstance();

        robot = new Robot(hardwareMap);
        ControlHubIdentifierUtil.setRobotName(hardwareMap, telemetry);
        robot.attachPedroFollower();
        robot.setAlliance(Alliance.BLUE);
        robot.initializeForTeleOp();

        Scheduler.reset();
        Scheduler.schedule(
                robot.drive.periodic(),
                robot.vision.periodic()
        );

        telemetry.addLine("=== Measurement Stdev Tuner ===");
        telemetry.addLine("1. Place robot at a known pose, tag 20 or 24 visible.");
        telemetry.addLine("2. Set Truth.actualX/Y/HeadingDeg in Dashboard.");
        telemetry.addLine("3. Press PLAY and hold still. Each fresh tag snapshot = 1 sample.");
        telemetry.addLine("4. Gamepad1 cross resets stats.");
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        Scheduler.execute();

        if (gamepad1.cross && !prevCross) {
            xVar.reset();
            yVar.reset();
            headingVar.reset();
            lastSampledNs = Long.MIN_VALUE;
        }
        prevCross = gamepad1.cross;

        VisionSubsystemLimelight.TagSnapshot snap = robot.vision.getLastSnapshot();
        if (snap != null) {
            Pose pose = snap.pedroPoseMT2 != null ? snap.pedroPoseMT2 : snap.pedroPoseMT1;
            if (pose != null && snap.capturedAtNs != lastSampledNs) {
                lastSampledNs = snap.capturedAtNs;

                double residualX = pose.getX() - Truth.actualXPedro;
                double residualY = pose.getY() - Truth.actualYPedro;
                double measuredHeadingDeg = Math.toDegrees(pose.getHeading());
                double residualHeadingDeg = AngleUnit.normalizeDegrees(
                        measuredHeadingDeg - Truth.actualHeadingDeg);

                xVar.update(residualX);
                yVar.update(residualY);
                headingVar.update(residualHeadingDeg);

                packet.put("StdevTuner/sample/residualX", residualX);
                packet.put("StdevTuner/sample/residualY", residualY);
                packet.put("StdevTuner/sample/residualHeadingDeg", residualHeadingDeg);
                packet.put("StdevTuner/sample/measuredX", pose.getX());
                packet.put("StdevTuner/sample/measuredY", pose.getY());
                packet.put("StdevTuner/sample/measuredHeadingDeg", measuredHeadingDeg);
            }
        }

        telemetry.addData("Samples", xVar.n());
        telemetry.addLine();
        telemetry.addLine("--- TRUTH (Pedro frame) ---");
        telemetry.addData("Truth X / Y / Heading",
                "%.2f in / %.2f in / %.2f°",
                Truth.actualXPedro, Truth.actualYPedro, Truth.actualHeadingDeg);
        telemetry.addLine();
        telemetry.addLine("--- RESIDUAL MEAN (bias) ---");
        telemetry.addData("mean X / Y / Heading",
                "%.4f in / %.4f in / %.4f°",
                xVar.mean(), yVar.mean(), headingVar.mean());
        telemetry.addLine();
        telemetry.addLine("--- STDEV (drop into createFollower R matrix) ---");
        telemetry.addData("stdev X", "%.4f in", xVar.stdDev());
        telemetry.addData("stdev Y", "%.4f in", yVar.stdDev());
        telemetry.addData("stdev Heading", "%.4f° (%.6f rad)",
                headingVar.stdDev(), Math.toRadians(headingVar.stdDev()));
        telemetry.addLine();
        telemetry.addLine("Use heading stdev in RADIANS in createFollower(...).");
        telemetry.addLine("Hold gamepad1 cross to reset.");

        packet.put("StdevTuner/samples", xVar.n());
        packet.put("StdevTuner/mean/x", xVar.mean());
        packet.put("StdevTuner/mean/y", yVar.mean());
        packet.put("StdevTuner/mean/headingDeg", headingVar.mean());
        packet.put("StdevTuner/stdev/x", xVar.stdDev());
        packet.put("StdevTuner/stdev/y", yVar.stdDev());
        packet.put("StdevTuner/stdev/headingDeg", headingVar.stdDev());
        packet.put("StdevTuner/stdev/headingRad", Math.toRadians(headingVar.stdDev()));

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.drive.stop();
        }
    }
}

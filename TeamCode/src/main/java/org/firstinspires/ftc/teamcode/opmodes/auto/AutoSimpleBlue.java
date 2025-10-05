//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;
//
///**
// * A simple autonomous routine that demonstrates using the DriveSubsystem in
// * autonomous mode with DevSim fallback.  When Pedro/NextFTC is integrated
// * this OpMode can be replaced with a command that follows a preplanned path.
// *
// * <p>This version is configured for the blue alliance.  It drives forward,
// * strafes to the right and then turns.  The durations assume the DevSim
// * default speeds (24 in/s, 120 deg/s).</p>
// */
//@Autonomous(name = "Auto Simple (Blue)", group = "Refactor")
//public class AutoSimpleBlue extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
//        // Create a PsiKit logger to record the autonomous run.  The log file
//        // will be stored in /sdcard/FIRST/PsiKitLogs.
//        org.firstinspires.ftc.teamcode.util.PsiKitAdapter logger = new org.firstinspires.ftc.teamcode.util.PsiKitAdapter();
//        logger.startSession();
//        TelemetryPublisher pub = new TelemetryPublisher(logger);
//
//        // Starting pose for blue alliance.  Adjust as needed.
//        drive.setPose(-12.0, -60.0, Math.toRadians(90.0));
//
//        waitForStart();
//        if (isStopRequested()) {
//            return;
//        }
//
//        // Drive forward 24 inches
//        runSegment(drive, pub, 0.0, 24.0);
//        // Strafe right 12 inches (field right is negative lx)
//        runSegment(drive, pub, -12.0, 0.0);
//        // Turn 90 degrees clockwise
//        runTurn(drive, pub, -90.0);
//
//        // Small pause at the end
//        sleep(300);
//
//        // End logging session
//        logger.stopSession();
//    }
//
//    /**
//     * Drive a straight segment in field coordinates.  The robot will move the
//     * given delta (dx, dy) over time using DevSim speeds.  Real odometry
//     * updates can be substituted when available.
//     */
//    private void runSegment(DriveSubsystem drive,
//                            TelemetryPublisher pub,
//                            double dxIn, double dyIn) {
//        // Duration based on distance and constant speed
//        double dist = Math.hypot(dxIn, dyIn);
//        double duration = dist / 24.0; // 24 in/s from Constants.DEV_SIM_SPEED_IPS
//        long endTime = System.currentTimeMillis() + (long) (duration * 1000.0);
//
//        // Normalise direction
//        double mag = dist;
//        double lx = 0.0;
//        double ly = 0.0;
//        if (mag > 1e-6) {
//            // Note: field frame: ly is forward, lx is left
//            lx = dxIn / mag;
//            ly = dyIn / mag;
//        }
//
//        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
//            drive.setSlowMode(false);
//            drive.drive(lx, ly, 0.0);
//            pub.publishDrive(drive, lx, ly, 0.0, false);
//            telemetry.update();
//            sleep(10);
//        }
//
//        // Brief stop to ensure the robot settles
//        drive.drive(0.0, 0.0, 0.0);
//    }
//
//    /**
//     * Turn the robot by a fixed angle in degrees using DevSim speeds.  A
//     * positive angle turns CCW; negative turns CW.
//     */
//    private void runTurn(DriveSubsystem drive,
//                         TelemetryPublisher pub,
//                         double deg) {
//        double duration = Math.abs(deg) / 120.0; // 120 deg/s from Constants.DEV_SIM_TURN_DPS
//        long endTime = System.currentTimeMillis() + (long) (duration * 1000.0);
//        double rx = Math.signum(deg) * 0.9; // spin quickly
//
//        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
//            drive.setSlowMode(false);
//            drive.drive(0.0, 0.0, rx);
//            pub.publishDrive(drive, 0.0, 0.0, rx, false);
//            telemetry.update();
//            sleep(10);
//        }
//
//        drive.drive(0.0, 0.0, 0.0);
//    }
//}

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryPublisher;

/**
 * A simple autonomous routine that mirrors the blue alliance routine for the red
 * alliance.  This OpMode demonstrates using the {@link DriveSubsystem} in
 * autonomous mode with DevSim fallback so that you can see the robot move
 * on the FTC Dashboard even when no hardware is attached.
 *
 * <p>When Pedro/NextFTC is integrated this file can be replaced with a
 * command that follows a preplanned path.  For now it drives forward,
 * strafes to the left and then turns counter‑clockwise.  Durations are
 * calculated from the DevSim speeds defined in {@code Constants}.</p>
 */
@Autonomous(name = "Auto Simple (Red)", group = "Refactor")
public class AutoSimpleRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        TelemetryPublisher pub = new TelemetryPublisher();

        // Starting pose for the red alliance.  X coordinate mirrored relative
        // to blue start.  Adjust values to match your field coordinate system.
        drive.setPose(12.0, -60.0, Math.toRadians(90.0));

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // Drive forward 24 inches (same as blue)
        runSegment(drive, pub, 0.0, 24.0);
        // Strafe left 12 inches (field left is positive lx)
        runSegment(drive, pub, 12.0, 0.0);
        // Turn 90 degrees CCW (positive angle)
        runTurn(drive, pub, 90.0);

        // Brief pause
        sleep(300);
    }

    /**
     * Drive a straight segment in field coordinates.  The robot will move the
     * given delta (dx, dy) over time using DevSim speeds.  Real odometry
     * updates can be substituted when available.
     */
    private void runSegment(DriveSubsystem drive,
                            TelemetryPublisher pub,
                            double dxIn, double dyIn) {
        double dist = Math.hypot(dxIn, dyIn);
        double duration = dist / 24.0; // 24 in/s from Constants.DEV_SIM_SPEED_IPS
        long endTime = System.currentTimeMillis() + (long) (duration * 1000.0);

        double mag = dist;
        double lx = 0.0;
        double ly = 0.0;
        if (mag > 1e-6) {
            lx = dxIn / mag;
            ly = dyIn / mag;
        }

        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            drive.setSlowMode(false);
            drive.drive(lx, ly, 0.0);
            pub.publishDrive(drive, lx, ly, 0.0, false);
            telemetry.update();
            sleep(10);
        }
        drive.drive(0.0, 0.0, 0.0);
    }

    /**
     * Turn the robot by a fixed angle in degrees using DevSim speeds.  A
     * positive angle turns CCW; negative turns CW.
     */
    private void runTurn(DriveSubsystem drive,
                         TelemetryPublisher pub,
                         double deg) {
        double duration = Math.abs(deg) / 120.0; // 120 deg/s from Constants.DEV_SIM_TURN_DPS
        long endTime = System.currentTimeMillis() + (long) (duration * 1000.0);
        double rx = Math.signum(deg) * 0.9;

        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            drive.setSlowMode(false);
            drive.drive(0.0, 0.0, rx);
            pub.publishDrive(drive, 0.0, 0.0, rx, false);
            telemetry.update();
            sleep(10);
        }
        drive.drive(0.0, 0.0, 0.0);
    }
}
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * TelemetryPublisher centralises the construction of telemetry packets for the
 * FTC Dashboard and AdvantageScope.  It publishes drive and flywheel state
 * variables on every loop so that they can be graphed or visualised.
 *
 * <p>The FTC Dashboard library will automatically aggregate packets from
 * multiple calls in the same loop, so publishing from both drive and
 * flywheel in one OpMode is safe.</p>
 */
public class TelemetryPublisher {

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final PsiKitAdapter logger;

    /**
     * Creates a TelemetryPublisher.  If a {@link PsiKitAdapter} is provided the
     * publisher will log all values to a CSV file in addition to sending them
     * to the FTC Dashboard.  Passing {@code null} disables file logging.
     *
     * @param logger an optional logger for persistent recordings
     */
    public TelemetryPublisher(PsiKitAdapter logger) {
        this.logger = logger;
    }

    /**
     * Convenience constructor that does not record to a file.  Only live
     * dashboard telemetry will be produced.
     */
    public TelemetryPublisher() {
        this(null);
    }

    /**
     * Publish drivetrain telemetry to the dashboard.  Includes joystick inputs,
     * the current pose estimate and slow mode state.
     *
     * @param drive    the drive subsystem to read the pose from
     * @param lx       left stick x
     * @param ly       left stick y
     * @param rx       right stick x (rotation)
     * @param slowMode whether slow mode is active
     */
    public void publishDrive(DriveSubsystem drive,
                             double lx, double ly, double rx,
                             boolean slowMode) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("lx", lx);
        p.put("ly", ly);
        p.put("rx", rx);
        p.put("slowMode", slowMode);

        Pose2D pose = drive.getPose();
        p.put("x_in", pose.getX(DistanceUnit.INCH));
        p.put("y_in", pose.getY(DistanceUnit.INCH));
        p.put("heading_deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS)));

        // Send to the live dashboard
        dash.sendTelemetryPacket(p);

        // Record to persistent log if enabled
        if (logger != null) {
            logger.recordNumber("drive_lx", lx);
            logger.recordNumber("drive_ly", ly);
            logger.recordNumber("drive_rx", rx);
            logger.recordBoolean("drive_slowMode", slowMode);
            logger.recordNumber("drive_x_in", pose.getX(DistanceUnit.INCH));
            logger.recordNumber("drive_y_in", pose.getY(DistanceUnit.INCH));
            logger.recordNumber("drive_heading_deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS)));
        }
    }

    /**
     * Publish flywheel telemetry.  Includes the target RPM, measured RPM,
     * control error and applied power.
     *
     * @param targetRpm desired wheel speed
     * @param rpm       current measured speed
     * @param power     last power sent to the motor
     * @param error     current error (target – measured)
     */
    public void publishFlywheel(double targetRpm, double rpm, double power, double error) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("fly_target_rpm", targetRpm);
        p.put("fly_rpm", rpm);
        p.put("fly_err", error);
        p.put("fly_power", power);
        dash.sendTelemetryPacket(p);
        if (logger != null) {
            logger.recordNumber("fly_target_rpm", targetRpm);
            logger.recordNumber("fly_rpm", rpm);
            logger.recordNumber("fly_err", error);
            logger.recordNumber("fly_power", power);
            // Flush immediately to minimise data loss if the robot shuts down
            logger.flush();
        }
    }
}

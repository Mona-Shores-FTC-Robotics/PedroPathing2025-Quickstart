package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

        DriveSubsystem.Pose2d pose = drive.getPose();
        p.put("x_in", pose.x);
        p.put("y_in", pose.y);
        p.put("heading_deg", Math.toDegrees(pose.headingRad));

        dash.sendTelemetryPacket(p);
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
    }
}

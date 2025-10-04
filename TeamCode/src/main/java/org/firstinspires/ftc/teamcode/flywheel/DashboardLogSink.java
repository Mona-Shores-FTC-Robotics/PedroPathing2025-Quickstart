package org.firstinspires.ftc.teamcode.flywheel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Sends a small packet to FTC Dashboard at ~N Hz.
 * AdvantageScope-Lite-FTC reads these live and plots them.
 */
public class DashboardLogSink implements FlywheelLogSink {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private long lastPublishTimeNanos = 0;

    @Override
    public void log(long timeMillis,
                    double rpm,
                    double targetRpm,
                    double motorPower,
                    double errorRpm,
                    double velocityTicksPerSecond,
                    double setpointTicksPerSecond) {

        long now = System.nanoTime();
        long periodNanos = (long) (1e9 / Math.max(1, FlywheelConfig.dashboardPublishHz));
        if (now - lastPublishTimeNanos < periodNanos) return; // throttle
        lastPublishTimeNanos = now;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("flywheel/target_rpm", targetRpm);
        packet.put("flywheel/rpm", rpm);
        packet.put("flywheel/error_rpm", errorRpm);
        packet.put("flywheel/power", motorPower);
        packet.put("flywheel/setpoint_tps", setpointTicksPerSecond);
        packet.put("flywheel/velocity_tps", velocityTicksPerSecond);
        dashboard.sendTelemetryPacket(packet);
    }
}

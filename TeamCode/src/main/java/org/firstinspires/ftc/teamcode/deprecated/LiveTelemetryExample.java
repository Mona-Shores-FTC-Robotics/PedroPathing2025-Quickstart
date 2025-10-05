package org.firstinspires.ftc.teamcode.deprecated;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiveTelemetryExample {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final Telemetry dsTelem;

    public LiveTelemetryExample(Telemetry dsTelem) {
        this.dsTelem = dsTelem;
    }

    public void update(double xInches, double yInches, double headingRad, double batteryV) {
        // 1) Driver Station text
        dsTelem.addData("x", xInches);
        dsTelem.addData("y", yInches);
        dsTelem.addData("heading(rad)", headingRad);
        dsTelem.addData("battery(V)", batteryV);
        dsTelem.update();

        // 2) FTC Dashboard packet (AdvantageScope Lite can read this live)
        TelemetryPacket p = new TelemetryPacket();
        p.put("x", xInches);
        p.put("y", yInches);
        p.put("heading(rad)", headingRad);
        p.put("battery(V)", batteryV);
        dash.sendTelemetryPacket(p);
    }
}

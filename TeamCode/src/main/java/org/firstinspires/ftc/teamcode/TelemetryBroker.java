package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryBroker {
    private final Telemetry ds;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    // PsiKit logger placeholder. AI coder will wire the real type from PsiKit docs.
    private final PsiKitAdapter psiLogger;

    public TelemetryBroker(Telemetry ds, PsiKitAdapter psiLogger) {
        this.ds = ds;
        this.psiLogger = psiLogger;
    }

    public void putNumber(String key, double value) {
        ds.addData(key, value);
        packet.put(key, value);
        psiLogger.recordNumber(key, value);
    }

    public void putBoolean(String key, boolean value) {
        ds.addData(key, value);
        packet.put(key, value ? 1 : 0); // Dashboard stores numbers
        psiLogger.recordBoolean(key, value);
    }

    public void flush() {
        ds.update();
        dash.sendTelemetryPacket(packet);
        psiLogger.flush();
        packet.clearLines(); // clear contents before next loop
    }
}

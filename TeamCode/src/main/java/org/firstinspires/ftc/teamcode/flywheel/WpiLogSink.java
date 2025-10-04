package org.firstinspires.ftc.teamcode.flywheel;

/**
 * Stub for persistent logging to AdvantageScope-replayable formats (e.g., .wpilog).
 * Replace the TODO sections with your chosen library (KoalaLog or PsiKit) calls.
 */
public class WpiLogSink implements FlywheelLogSink {

    private final boolean enabled;

    public WpiLogSink(boolean enable) {
        this.enabled = enable;
        // TODO: initialize your logger here (open file / set up topics)
    }

    @Override
    public void periodic() {
        if (!enabled) return;
        // TODO: flush occasionally if your library requires it
    }

    @Override
    public void log(long timeMillis,
                    double rpm,
                    double targetRpm,
                    double motorPower,
                    double errorRpm,
                    double velocityTicksPerSecond,
                    double setpointTicksPerSecond) {
        if (!enabled) return;
        // TODO: publish values to your logger here (non-blocking!)
    }

    @Override
    public void close() {
        if (!enabled) return;
        // TODO: close your logger here
    }
}

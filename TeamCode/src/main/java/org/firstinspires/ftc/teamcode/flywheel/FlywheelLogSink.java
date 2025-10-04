package org.firstinspires.ftc.teamcode.flywheel;

/** Logging target interface (live plots, persistent logs, etc.). */
public interface FlywheelLogSink {
    default void periodic() {}  // optional, called ~50 Hz from UI thread
    void log(long timeMillis,
             double rpm,
             double targetRpm,
             double motorPower,
             double errorRpm,
             double velocityTicksPerSecond,
             double setpointTicksPerSecond);
    default void close() {}
}

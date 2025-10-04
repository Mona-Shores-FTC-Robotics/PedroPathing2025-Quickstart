package org.firstinspires.ftc.teamcode.flywheel;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Live-tunable knobs shown in Panels.
 * Units are included in each name for clarity.
 */
@Configurable
public class FlywheelConfig {

    // --- PID gains (error = setpointTicksPerSecond - measuredTicksPerSecond) ---
    public static double kP = 0.0008;    // power per (tick/s) of error
    public static double kI = 0.0002;    // power per (tick/s)*s
    public static double kD = 0.0001;    // power per (tick/s)/s (derivative on measurement)

    // --- Feedforward (open-loop) ---
    public static double kS = 0.05;      // static power to overcome friction (0..1)
    public static double kV = 0.00025;   // power per (tick/s) of setpoint
    public static double kA = 0.0;       // power per (tick/s^2) of estimated accel (usually 0)

    // --- PID behavior ---
    public static double integralClampAbs = 0.25; // hard clamp on integrator (prevents windup)
    public static double powerMin = 0.0;         // flywheels usually do not brake
    public static double powerMax = 1.0;

    // --- Setpoint shaping (gentle ramping to reduce brownouts/overshoot) ---
    public static double maxSetpointAccelTicksPerSecond2 = 4000.0;

    // --- Targets (RPM) for testing from the gamepad ---
    public static double targetRpm = 3000.0;
    public static double targetStepRpm = 250.0;

    // --- Filtering & publish rate ---
    public static double velocityEmaAlpha = 0.20; // 0..1; higher = smoother but laggier
    public static int dashboardPublishHz = 50;    // AdvantageScope-Lite via FTC Dashboard
}

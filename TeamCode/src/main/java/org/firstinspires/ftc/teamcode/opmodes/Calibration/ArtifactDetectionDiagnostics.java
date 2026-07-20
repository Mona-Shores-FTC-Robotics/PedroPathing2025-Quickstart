package org.firstinspires.ftc.teamcode.opmodes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.LaneSample;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Real-time artifact detection diagnostics.
 *
 * Shows live sensor readings, classification decisions, and noise analysis
 * to help debug false positives/negatives.
 *
 * Controls:
 * - D-pad Up/Down: Select lane to focus on (or ALL)
 * - A: Freeze current readings for inspection
 * - X: Clear history and reset
 * - Left Stick Y: Scroll through historical samples
 */
@Disabled
@TeleOp(name = "Artifact Detection Diagnostics", group = "Tuning")
public class ArtifactDetectionDiagnostics extends LinearOpMode {

    private IntakeSubsystem intake;
    private FtcDashboard dashboard;
    private LauncherLane focusLane = null; // null = show all lanes
    private boolean frozen = false;
    private final ElapsedTime runtime = new ElapsedTime();

    // History tracking for noise analysis
    private static final int HISTORY_SIZE = 50; // Keep last 50 samples per lane
    private final List<TimestampedSample> leftHistory = new ArrayList<>();
    private final List<TimestampedSample> centerHistory = new ArrayList<>();
    private final List<TimestampedSample> rightHistory = new ArrayList<>();

    private static class TimestampedSample {
        final double timestampSec;
        final LaneSample sample;
        final ArtifactColor detectedColor;
        final String reason;

        TimestampedSample(double timestamp, LaneSample sample, ArtifactColor color, String reason) {
            this.timestampSec = timestamp;
            this.sample = sample;
            this.detectedColor = color;
            this.reason = reason != null ? reason : "unknown";
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Artifact Detection Diagnostics...");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();
        org.firstinspires.ftc.teamcode.util.RobotProfile profile =
                org.firstinspires.ftc.teamcode.util.RobotProfile.forCurrent();
        intake = new IntakeSubsystem(hardwareMap, profile.gate, profile.lanePresence);
        intake.initialize();

        telemetry.addLine("✓ Ready!");
        telemetry.addLine();
        telemetry.addLine("This OpMode shows live sensor readings and helps");
        telemetry.addLine("diagnose false positives/negatives.");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean lastA = false;
        boolean lastX = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (opModeIsActive()) {
            // Update intake sensors
            if (!frozen) {
                intake.periodic();
                recordHistory();
            }

            // Controls
            if (gamepad1.a && !lastA) {
                frozen = !frozen;
            }
            lastA = gamepad1.a;

            if (gamepad1.x && !lastX) {
                clearHistory();
            }
            lastX = gamepad1.x;

            if (gamepad1.dpad_up && !lastDpadUp) {
                focusLane = cycleLane(focusLane, 1);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                focusLane = cycleLane(focusLane, -1);
            }
            lastDpadDown = gamepad1.dpad_down;

            // Display diagnostics
            displayDiagnostics();
        }
    }

    private void recordHistory() {
        double timestamp = runtime.seconds();

        LaneSample left = intake.getLaneSample(LauncherLane.LEFT);
        LaneSample center = intake.getLaneSample(LauncherLane.CENTER);
        LaneSample right = intake.getLaneSample(LauncherLane.RIGHT);

        // Extract classification reason from telemetry (this is logged by IntakeSubsystem)
        // For now, use the detected color as a proxy
        String leftReason = left.color == null ? "none" : left.color.name().toLowerCase(Locale.US);
        String centerReason = center.color == null ? "none" : center.color.name().toLowerCase(Locale.US);
        String rightReason = right.color == null ? "none" : right.color.name().toLowerCase(Locale.US);

        addToHistory(leftHistory, timestamp, left, left.color, leftReason);
        addToHistory(centerHistory, timestamp, center, center.color, centerReason);
        addToHistory(rightHistory, timestamp, right, right.color, rightReason);
    }

    private void addToHistory(List<TimestampedSample> history, double timestamp,
                              LaneSample sample, ArtifactColor color, String reason) {
        history.add(new TimestampedSample(timestamp, sample, color, reason));
        if (history.size() > HISTORY_SIZE) {
            history.remove(0);
        }
    }

    private void clearHistory() {
        leftHistory.clear();
        centerHistory.clear();
        rightHistory.clear();
    }

    private LauncherLane cycleLane(LauncherLane current, int direction) {
        if (current == null) {
            return direction > 0 ? LauncherLane.LEFT : LauncherLane.RIGHT;
        }

        LauncherLane[] lanes = LauncherLane.values();
        int index = current.ordinal() + direction;

        if (index < 0) {
            return null; // Wrap to "ALL"
        } else if (index >= lanes.length) {
            return null; // Wrap to "ALL"
        } else {
            return lanes[index];
        }
    }

    private void displayDiagnostics() {
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addLine("=== ARTIFACT DETECTION DIAGNOSTICS ===");
        telemetry.addLine();
        telemetry.addData("Status", frozen ? "FROZEN (press A to unfreeze)" : "LIVE");
        telemetry.addData("Focus", focusLane == null ? "ALL LANES (D-pad ↑↓)" : focusLane.name() + " (D-pad ↑↓)");
        telemetry.addData("Clear History", "Press X");
        telemetry.addLine();

        packet.put("Status", frozen ? "FROZEN" : "LIVE");
        packet.put("Focus Lane", focusLane == null ? "ALL" : focusLane.name());

        if (focusLane == null) {
            // Show all lanes summary
            displayLaneSummary(LauncherLane.LEFT, leftHistory, packet);
            displayLaneSummary(LauncherLane.CENTER, centerHistory, packet);
            displayLaneSummary(LauncherLane.RIGHT, rightHistory, packet);
        } else {
            // Show detailed view for focused lane
            List<TimestampedSample> history = getHistoryForLane(focusLane);
            displayDetailedLane(focusLane, history, packet);
        }

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    private void displayLaneSummary(LauncherLane lane, List<TimestampedSample> history, TelemetryPacket packet) {
        String laneName = lane.name();
        LaneSample current = intake.getLaneSample(lane);
        ArtifactColor currentColor = intake.getLaneColor(lane);

        telemetry.addLine(String.format("--- %s ---", laneName));

        if (!current.sensorPresent) {
            telemetry.addData("  Sensor", "NOT PRESENT");
            packet.put(laneName + "/Error", "Sensor NOT PRESENT");
            return;
        }

        // Current state
//        telemetry.addData("  Detected", "%s (conf: %.2f)", current.color, current.confidence);
        telemetry.addData("  Distance", "%.1f cm %s",
            current.distanceCm,
            current.presenceDetected ? "✓" : "✗");
        telemetry.addData("  HSV", "H:%.0f° S:%.2f V:%.2f",
            current.hue, current.saturation, current.value);

        packet.put(laneName + "/Detected", current.color != null ? current.color.name() : "NONE");
//        packet.put(laneName + "/Confidence", current.confidence);
        packet.put(laneName + "/Distance_cm", current.distanceCm);
        packet.put(laneName + "/Presence_Detected", current.presenceDetected);
        packet.put(laneName + "/Hue", current.hue);
        packet.put(laneName + "/Saturation", current.saturation);
        packet.put(laneName + "/Value", current.value);

        // Noise analysis from history
        if (history.size() >= 10) {
            NoiseStats stats = computeNoiseStats(history);
            telemetry.addData("  Stability", "%.1f%% (last %d samples)",
                stats.stabilityPercent, history.size());
            telemetry.addData("  Flicker Count", "%d transitions", stats.transitionCount);

            packet.put(laneName + "/Noise/Stability_Percent", stats.stabilityPercent);
            packet.put(laneName + "/Noise/Transitions", stats.transitionCount);
            packet.put(laneName + "/Noise/Distance_Stddev_cm", stats.distanceStddev);

            if (stats.stabilityPercent < 70.0) {
                telemetry.addLine("  ⚠ HIGH NOISE - tune thresholds!");
            }
        }

        telemetry.addLine();
    }

    private void displayDetailedLane(LauncherLane lane, List<TimestampedSample> history, TelemetryPacket packet) {
        String laneName = lane.name();
        LaneSample current = intake.getLaneSample(lane);

        telemetry.addLine(String.format("=== %s LANE (DETAILED) ===", laneName));
        telemetry.addLine();

        if (!current.sensorPresent) {
            telemetry.addData("Sensor", "NOT PRESENT");
            return;
        }

        // Current reading - detailed
        telemetry.addLine("--- Current Reading ---");
        telemetry.addData("Detected Color", "%s", current.color);
//        telemetry.addData("Confidence", "%.2f", current.confidence);
        telemetry.addData("Distance", "%.2f cm (present: %s)", current.distanceCm, current.presenceDetected);
        telemetry.addData("Hue", "%.1f°", current.hue);
        telemetry.addData("Saturation", "%.3f", current.saturation);
        telemetry.addData("Value", "%.3f", current.value);
        telemetry.addData("RGB (scaled)", "R:%d G:%d B:%d",
            current.scaledRed, current.scaledGreen, current.scaledBlue);
        telemetry.addData("RGB (norm)", "R:%.2f G:%.2f B:%.2f",
            current.normalizedRed, current.normalizedGreen, current.normalizedBlue);

        packet.put(laneName + "/Current/Color", current.color != null ? current.color.name() : "NONE");
//        packet.put(laneName + "/Current/Confidence", current.confidence);
        packet.put(laneName + "/Current/Distance_cm", current.distanceCm);
        packet.put(laneName + "/Current/Presence_Detected", current.presenceDetected);
        packet.put(laneName + "/Current/Hue", current.hue);
        packet.put(laneName + "/Current/Saturation", current.saturation);
        packet.put(laneName + "/Current/Value", current.value);

        // Noise analysis
        if (history.size() >= 10) {
            telemetry.addLine();
            telemetry.addLine("--- Noise Analysis ---");
            NoiseStats stats = computeNoiseStats(history);

            telemetry.addData("Sample Count", "%d", history.size());
            telemetry.addData("Stability", "%.1f%%", stats.stabilityPercent);
            telemetry.addData("Transitions", "%d", stats.transitionCount);
            telemetry.addData("Distance Stddev", "%.2f cm", stats.distanceStddev);
            telemetry.addData("Hue Stddev", "%.1f°", stats.hueStddev);
            telemetry.addData("Sat Stddev", "%.3f", stats.satStddev);

            packet.put(laneName + "/Noise/Sample_Count", history.size());
            packet.put(laneName + "/Noise/Stability_Percent", stats.stabilityPercent);
            packet.put(laneName + "/Noise/Transitions", stats.transitionCount);
            packet.put(laneName + "/Noise/Distance_Stddev_cm", stats.distanceStddev);
            packet.put(laneName + "/Noise/Hue_Stddev", stats.hueStddev);
            packet.put(laneName + "/Noise/Saturation_Stddev", stats.satStddev);

            // Diagnosis
            telemetry.addLine();
            if (stats.stabilityPercent < 50.0) {
                telemetry.addLine("⚠ CRITICAL: Very unstable detection");
                telemetry.addLine("  → Increase debounce (consecutiveConfirmations)");
                telemetry.addLine("  → Check sensor gain and lighting");
            } else if (stats.stabilityPercent < 70.0) {
                telemetry.addLine("⚠ WARNING: Noisy detection");
                telemetry.addLine("  → Tune presence thresholds (distance/sat/val/hue)");
                telemetry.addLine("  → Increase debounce (consecutiveConfirmations)");
            } else {
                telemetry.addLine("✓ GOOD: Stable detection");
            }

            if (stats.distanceStddev > 1.0) {
                telemetry.addLine("⚠ Distance readings very noisy");
                telemetry.addLine("  → Check sensor mounting / vibration");
            }

            // Recent transition history
            telemetry.addLine();
            telemetry.addLine("--- Recent Transitions (last 10) ---");
            displayRecentTransitions(history, 10);
        } else {
            telemetry.addLine();
            telemetry.addLine("(Collecting samples... need 10+)");
        }
    }

    private void displayRecentTransitions(List<TimestampedSample> history, int count) {
        int displayed = 0;
        ArtifactColor lastColor = null;

        // Walk backwards through history to find transitions
        for (int i = history.size() - 1; i >= 0 && displayed < count; i--) {
            TimestampedSample sample = history.get(i);

            if (lastColor == null || !sample.detectedColor.equals(lastColor)) {
                telemetry.addData("  %.1fs", String.valueOf(sample.timestampSec),
                    "%s → %s", lastColor != null ? lastColor : "?", sample.detectedColor);
                displayed++;
            }

            lastColor = sample.detectedColor;
        }
    }

    private List<TimestampedSample> getHistoryForLane(LauncherLane lane) {
        switch (lane) {
            case LEFT: return leftHistory;
            case CENTER: return centerHistory;
            case RIGHT: return rightHistory;
            default: return new ArrayList<>();
        }
    }

    private static class NoiseStats {
        double stabilityPercent;
        int transitionCount;
        double distanceStddev;
        double hueStddev;
        double satStddev;
    }

    private NoiseStats computeNoiseStats(List<TimestampedSample> history) {
        NoiseStats stats = new NoiseStats();

        if (history.isEmpty()) {
            return stats;
        }

        // Count transitions (color changes)
        ArtifactColor lastColor = null;
        int transitions = 0;
        int stableCount = 0; // Samples that match majority color

        // Find majority color
        int greenCount = 0, purpleCount = 0, noneCount = 0, otherCount = 0;
        for (TimestampedSample sample : history) {
            if (sample.detectedColor == ArtifactColor.GREEN) greenCount++;
            else if (sample.detectedColor == ArtifactColor.PURPLE) purpleCount++;
            else if (sample.detectedColor == ArtifactColor.NONE) noneCount++;
            else otherCount++;
        }

        int maxCount = Math.max(Math.max(greenCount, purpleCount), Math.max(noneCount, otherCount));
        ArtifactColor majorityColor;
        if (maxCount == greenCount) majorityColor = ArtifactColor.GREEN;
        else if (maxCount == purpleCount) majorityColor = ArtifactColor.PURPLE;
        else if (maxCount == noneCount) majorityColor = ArtifactColor.NONE;
        else majorityColor = ArtifactColor.UNKNOWN;

        // Count transitions and stability
        for (TimestampedSample sample : history) {
            if (lastColor != null && !sample.detectedColor.equals(lastColor)) {
                transitions++;
            }
            if (sample.detectedColor.equals(majorityColor)) {
                stableCount++;
            }
            lastColor = sample.detectedColor;
        }

        stats.transitionCount = transitions;
        stats.stabilityPercent = (stableCount * 100.0) / history.size();

        // Compute standard deviations
        stats.distanceStddev = computeStddev(history, s -> (float) s.sample.distanceCm);
        stats.hueStddev = computeStddev(history, s -> s.sample.hue);
        stats.satStddev = computeStddev(history, s -> s.sample.saturation);

        return stats;
    }

    private double computeStddev(List<TimestampedSample> history, SampleExtractor extractor) {
        if (history.isEmpty()) return 0.0;

        // Compute mean
        double sum = 0.0;
        int validCount = 0;
        for (TimestampedSample sample : history) {
            float val = extractor.extract(sample);
            if (!Float.isNaN(val) && !Float.isInfinite(val)) {
                sum += val;
                validCount++;
            }
        }

        if (validCount == 0) return 0.0;
        double mean = sum / validCount;

        // Compute variance
        double varianceSum = 0.0;
        for (TimestampedSample sample : history) {
            float val = extractor.extract(sample);
            if (!Float.isNaN(val) && !Float.isInfinite(val)) {
                double diff = val - mean;
                varianceSum += diff * diff;
            }
        }

        double variance = varianceSum / validCount;
        return Math.sqrt(variance);
    }

    @FunctionalInterface
    private interface SampleExtractor {
        float extract(TimestampedSample sample);
    }
}

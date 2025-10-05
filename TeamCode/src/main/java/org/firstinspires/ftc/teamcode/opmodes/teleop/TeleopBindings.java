// src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/TeleopBindings.java
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

/**
 * Encapsulates all driver/operator button bindings for TeleOp.
 * Use {@link #update()} each loop and {@link #publishHelp(Telemetry)} before
 * telemetry.update() to display control hints when help is toggled.
 */
public class TeleopBindings {

    /** Holds a description and the corresponding binding button. */
    private static class BindingEntry {
        final String description;
        final Button button;

        BindingEntry(String description, Button button) {
            this.description = description;
            this.button = button;
        }
    }

    private final List<BindingEntry> driverBindings = new ArrayList<>();
    private final List<BindingEntry> operatorBindings = new ArrayList<>();
    private boolean showDriverHelp = false;
    private boolean showOperatorHelp = false;

    public TeleopBindings(GamepadEx driver, GamepadEx operator, Robot robot) {

        // Driver bindings ----------------------------------------------------


        // Toggle flywheel on A: press once to start, press again to stop.
        Button flyToggle = driver.a().toggleOnBecomesTrue();
        flyToggle.whenBecomesTrue(() -> robot.flywheel.setTargetRpm(Constants.FLY_TEST_RPM));
        flyToggle.whenBecomesFalse(robot.flywheel::stop);
        driverBindings.add(new BindingEntry("Driver A: toggle flywheel", flyToggle));

        // Toggle slow mode on right bumper (press once to engage, again to disengage).
        Button slowModeToggle = driver.rightBumper().toggleOnBecomesTrue();
        slowModeToggle.whenBecomesTrue(() -> robot.drive.setSlowMode(true));
        slowModeToggle.whenBecomesFalse(() -> robot.drive.setSlowMode(false));
        driverBindings.add(new BindingEntry("Driver RB: toggle slow mode", slowModeToggle));

        // Help toggle for driver: use the Options button to show/hide driver controls.
        Button driverHelpToggle = driver.options().toggleOnBecomesTrue();
        driverHelpToggle.whenBecomesTrue(() -> showDriverHelp = true);
        driverHelpToggle.whenBecomesFalse(() -> showDriverHelp = false);

        // Operator bindings --------------------------------------------------
        // (Add operator controls here as you build more subsystems.)

        // Help toggle for operator.
        Button operatorHelpToggle = operator.options().toggleOnBecomesTrue();
        operatorHelpToggle.whenBecomesTrue(() -> showOperatorHelp = true);
        operatorHelpToggle.whenBecomesFalse(() -> showOperatorHelp = false);
    }

    /** Update all bindings.  Call this once per loop in your OpMode. */
    public void update() {
        BindingManager.update();
    }

    /** Reset all bindings.  Call this in OpMode.stop(). */
    public void reset() {
        BindingManager.reset();
        showDriverHelp = false;
        showOperatorHelp = false;
    }

    /**
     * Add button descriptions to the telemetry if help is toggled on.
     * Call this just before telemetry.update() in your OpMode loop.
     */
    public void publishHelp(Telemetry telemetry) {
        if (showDriverHelp) {
            driverBindings.forEach(entry -> telemetry.addLine(entry.description));
        }
        if (showOperatorHelp) {
            operatorBindings.forEach(entry -> telemetry.addLine(entry.description));
        }
    }
}

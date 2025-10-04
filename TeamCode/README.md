# DECODE Command‑Based Scaffold

This branch introduces a clean, teachable command‑based architecture for the
FTC DECODE season built on top of the existing U‑Chassis code.  The goal is
to separate hardware management (subsystems) from behaviour (commands) and
provide a clear path toward integrating advanced libraries such as Pedro
Pathing and NextFTC.

## Project Structure

The core modules are:

| Path | Purpose |
| --- | --- |
| `subsystems/DriveSubsystem.java` | Handles mecanum drive, field‑centric control with automatic heading alignment, slow‑mode toggling and simulated pose integration when DevSim is active. |
| `subsystems/FlywheelSubsystem.java` | Implements a bang‑bang + feedforward + proportional control loop for a single flywheel.  This subsystem exposes a simple API to set a target RPM and handles all control internally. |
| `opmodes/teleop/FieldCentricTeleOp.java` | Binds gamepad inputs to the drive and flywheel subsystems, enabling slow mode with the right bumper and flywheel spin‑up with the A/B buttons. |
| `opmodes/auto/AutoSimpleBlue.java` & `AutoSimpleRed.java` | Simple autonomous routines for the blue and red alliances.  These files demonstrate how to drive predetermined segments using the drive subsystem.  When DevSim is enabled they animate robot motion on the FTC Dashboard; once Pedro integration is added these routines can be replaced with a path follower. |
| `util/TelemetryPublisher.java` | A helper for publishing robot pose, drive inputs and flywheel data to the FTC Dashboard. |
| `Constants.java` | Consolidates hardware names, tuning constants and the `DEV_SIM_ENABLED` flag. |

## DevSim

When `Constants.DEV_SIM_ENABLED` is `true` the drive subsystem integrates joystick inputs into a simulated pose.  This allows you to run your teleop or auto code on a device with no motors attached and still see the robot move on the dashboard.  The simulated speeds (`DEV_SIM_SPEED_IPS` and `DEV_SIM_TURN_DPS`) can be tuned to approximate your real robot’s behaviour.

To disable DevSim when running on the actual robot, set

```java
Constants.DEV_SIM_ENABLED = false;
```

or change the default value in `Constants.java`.  When DevSim is off, be sure to implement your real odometry updates in `DriveSubsystem.updateOdometryDevSim()`.

## Live Tuning via FTC Dashboard

This scaffold marks the `Constants` class with `@Config` from the FTC Dashboard library.  All non‑final public static fields (such as `HEADING_KP`, `SLOW_MODE_SCALE`, `FLY_KF`, etc.) become editable on the **Config** tab of the dashboard.  Use this feature to adjust gains and thresholds without redeploying your code.  Changes take effect immediately and persist until the next app restart.

## Logging

### Live Telemetry

`TelemetryPublisher` sends values to the FTC Dashboard on every loop.  You can monitor:

* `drive_lx`, `drive_ly`, `drive_rx` — joystick commands
* `drive_x_in`, `drive_y_in`, `drive_heading_deg` — robot pose estimates (inches and degrees)
* `fly_target_rpm`, `fly_rpm`, `fly_err`, `fly_power` — target flywheel speed, measured speed, error and applied motor power

Plots and tables of these values are available in the dashboard and AdvantageScope when connected live.

### Persistent Logs (PsiKit Adapter)

For offline analysis, the scaffold includes a `PsiKitAdapter` that writes all telemetry to a CSV file on the robot.  A new log file is created when an OpMode is initialised and closed when it stops.  Files are saved under `/sdcard/FIRST/PsiKitLogs` with names like `log_20251004_153000.csv`.  Each row contains a timestamp, a key and a value.  You can drag these CSV files into AdvantageScope and replay your runs.  Because the adapter matches the same keys used for live telemetry, you will see identical graphs.

If you later choose to integrate the official PsiKit library, you can replace the simple adapter with the real implementation without changing the rest of your code.

## FTC Dashboard and AdvantageScope

The scaffold includes the FTC Dashboard library (`ftc-dashboard 0.4.8`).  To view telemetry:

1. Connect your programming laptop to the Robot Controller’s Wi‑Fi network.
2. Open a web browser and navigate to `http://192.168.49.1:8080/dash`.
3. Select the **Telemetry** tab to see live numbers for the drive inputs, pose, target RPM and current RPM.
4. Expand the **Graph** section to plot variables over time (e.g., flywheel RPM vs. target RPM).

AdvantageScope can connect to the same data stream via the NetworkTables 4 protocol.  Add a new **NT4** source using the Robot Controller’s address and watch the 2D field plot update as your robot moves (even in DevSim).  This is a great way to tune heading gains and verify autonomous paths.

## Adding Pedro and NextFTC

This scaffold does not yet depend on Pedro Pathing or NextFTC.  The placeholder `Constants.createPedroFollower()` method will throw an exception if called.  When you are ready to integrate Pedro/NextFTC:

1. Add the appropriate dependencies to `TeamCode/build.gradle` (see the Pedro/NextFTC documentation for version numbers).
2. Replace the DevSim segments in `AutoSimpleBlue` and `AutoSimpleRed` with calls to a Pedro path follower.
3. Remove or bypass DevSim by setting `DEV_SIM_ENABLED` to false and implementing odometry updates.

With this structure in place, you can focus on tuning and testing your robot’s behaviour rather than fighting a monolithic code base.  Happy coding!
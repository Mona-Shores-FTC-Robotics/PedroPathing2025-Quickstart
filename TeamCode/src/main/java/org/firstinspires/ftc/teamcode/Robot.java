// src/main/java/org/firstinspires/ftc/teamcode/Robot.java
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

/** Simple container for all of the robot's subsystems. */
public class Robot {
    public final DriveSubsystem drive;
    public final FlywheelSubsystem flywheel;

    public Robot(HardwareMap hardwareMap) {
        drive = new DriveSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
    }
}

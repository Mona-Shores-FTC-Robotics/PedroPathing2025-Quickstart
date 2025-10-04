package org.firstinspires.ftc.teamcode;

public final class Fields {
    private Fields() {}

    // Robot state
    public static final String POSE_X_IN = "Pose x";                 // inches
    public static final String POSE_Y_IN = "Pose y";                 // inches
    public static final String POSE_HEADING_RAD = "Pose heading";    // radians (or use "Pose heading (deg)")

    // Drive
    public static final String DRIVE_TARGET_VEL = "drive/targetVel_ips"; // inches per second
    public static final String DRIVE_ACTUAL_VEL = "drive/actualVel_ips";

    // Power
    public static final String BATTERY_V = "power/battery_V";

    // Mechanisms
    public static final String FLYWHEEL_RPM = "flywheel/rpm";
    public static final String LIFT_POS_IN = "lift/pos_in";
    public static final String LIFT_AT_TARGET = "lift/atTarget";

    // Vision example
    public static final String TAGS_VISIBLE = "vision/tagsVisible";
}

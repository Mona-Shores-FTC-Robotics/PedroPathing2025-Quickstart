// org.firstinspires.ftc.teamcode.util.RobotState
package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import java.util.concurrent.atomic.AtomicReference;

public class RobotState {
    private static final AtomicReference<Pose> handoffPose = new AtomicReference<>(null);

    public static void setHandoffPose(Pose p) { handoffPose.set(p); }
    public static Pose takeHandoffPose()      { return handoffPose.getAndSet(null); } // consume-once
}

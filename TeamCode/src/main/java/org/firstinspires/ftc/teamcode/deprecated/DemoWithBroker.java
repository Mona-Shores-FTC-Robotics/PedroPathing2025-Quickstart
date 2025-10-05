package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Fields;

@TeleOp(name="DemoWithBroker")
public class DemoWithBroker extends LinearOpMode {
    private TelemetryBroker broker;
    private PsiKitAdapter psi;

    @Override public void runOpMode() {
        psi = new PsiKitAdapter();
        broker = new TelemetryBroker(telemetry, psi);

        psi.startSession("teleop"); // new log per run

        waitForStart();
        while (opModeIsActive()) {
//            double xIn = /* your pose x in inches */;
//            double yIn = /* your pose y in inches */;
//            double headingRad = /* your heading in radians */;
//            double batteryV = /* read from hub */;
//
//            broker.putNumber(Fields.POSE_X_IN, xIn);
//            broker.putNumber(Fields.POSE_Y_IN, yIn);
//            broker.putNumber(Fields.POSE_HEADING_RAD, headingRad);
//            broker.putNumber(Fields.BATTERY_V, batteryV);

            // Example mechanism fields
            broker.putNumber(Fields.FLYWHEEL_RPM, /* rpm */ 0);
            broker.putNumber(Fields.LIFT_POS_IN, /* inches */ 0);
            broker.putBoolean(Fields.LIFT_AT_TARGET, /* at target */ false);

            broker.flush();
        }
        psi.close();
    }
}

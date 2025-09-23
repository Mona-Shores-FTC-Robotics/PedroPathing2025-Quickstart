package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.logging.Filter;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.05,0,0.0,0.11))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0,0,0.000,0.08))

            .headingPIDFCoefficients(new PIDFCoefficients(0,0,0.0,0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0,0,0.000,0))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0.0,0, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0, 0))

            .mass(5);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("rf") //  Motor Port 0
            .rightRearMotorName("rb") //   Motor Port 1
            .leftRearMotorName("lb") //    Motor Port 2
            .leftFrontMotorName("lf") //   Motor Port 3
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.25)
            .strafePodX(5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") // I2C Bus 1 Expansion
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)

                .build();
    }
}

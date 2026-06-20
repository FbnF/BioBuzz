package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Constants {


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.25)
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.00001,0.055))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.75,0,0.00001,0.6,0))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .forwardZeroPowerAcceleration(-26.2366605549332)
            .lateralZeroPowerAcceleration(-60.326310794020785)
            .centripetalScaling(0.0005)
            .lateralZeroPowerAcceleration(-60.780323877099995);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 1.2);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(73.10214152906667)
            .yVelocity(53.5788395452)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);




    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.002984549)
            .strafeTicksToInches(-0.00297651915) //0.002971218280499472
            .turnTicksToInches(0.001289916917918203)
            .leftPodY(6.5)
            .rightPodY(-6.5)
            .strafePodX(2.5)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightFront")
            .strafeEncoder_HardwareMapName("leftBack")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

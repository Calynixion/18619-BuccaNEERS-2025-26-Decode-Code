package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .forwardZeroPowerAcceleration(-49.09805325)
            .lateralZeroPowerAcceleration(-83.11857244)
            .drivePIDFSwitch(3)
            .headingPIDFSwitch(3)
            .translationalPIDFSwitch(3)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(.15,0,0.001,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.25,0,0.01,0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.01,0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.3,0,0.1,0.0005))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.000005,0.6,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0.01))
            .centripetalScaling(0.02);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FrontRightM")
            .rightRearMotorName("BackRightM")
            .leftRearMotorName("BackLeftM")
            .leftFrontMotorName("FrontLeftM")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(55.849439)
            .yVelocity(40.24125065);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.00297692)
            .strafeTicksToInches(0.0029647579)
            .turnTicksToInches(0.0030096411)
            .leftPodY(8.4)
            .rightPodY(-8.4)
            .strafePodX(-5.75)
            .leftEncoder_HardwareMapName("BackRightM")
            .rightEncoder_HardwareMapName("FrontLeftM")
            .strafeEncoder_HardwareMapName("FrontRightM")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

     /*
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardTicksToInches(0.00297692)
            .strafeTicksToInches(0.0029647579)
            //.turnTicksToInches(0.0030096411)
            //.leftPodY(8.4)
            .rightPodY(-8.4)
            .strafePodX(-5.75)
            .leftEncoder_HardwareMapName("BackRightM")
            .rightEncoder_HardwareMapName("FrontLeftM")
            .strafeEncoder_HardwareMapName("FrontRightM")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);
    */


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.95, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}

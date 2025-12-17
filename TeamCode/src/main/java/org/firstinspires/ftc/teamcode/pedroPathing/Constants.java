package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6);
    public static MecanumConstants driveConstants = new MecanumConstants()
                    .maxPower(0.5) //TODO
                    .rightFrontMotorName("drft")
                    .rightRearMotorName("drspa")
                    .leftRearMotorName("stspa")
                    .leftFrontMotorName("stft")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .offset(new SparkFunOTOS.Pose2D(0,-0.0787401575,Math.PI/2))
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(-0.985)/// LASA PE -0.985 (minimul pe care il aveam era 0.97 si max era sub 1
            .angularScalar(1);///LASA PE 1

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)//bunu
                .mecanumDrivetrain(driveConstants)//bunu
                .build();
    }

}

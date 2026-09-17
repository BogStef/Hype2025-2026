package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Gyro {
    public IMU imu;


    private GoBildaPinpointDriver pinpoint;
    public Gyro(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu_expension");
        imu.initialize(
        new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN))
        );

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

    }


    public double getY(){
        return pinpoint.getPosY(DistanceUnit.CM);
    }

    public double getHeading(){
        //return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        pinpoint.update();
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }


//   // public Acceleration getAcceleration(){
//        return imu.getAcceleration();
//    }
}
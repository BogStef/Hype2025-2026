

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import java.util.List;

@TeleOp(name = "❤(｡◕‿‿◕｡)✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟✟", group = "TeleOp")
public class teleoperationimagination extends LinearOpMode {
    public DcMotorEx drft, stft, drspa, stspa;
    public DcMotorEx tun, intake1, intake2;
    //public IMU imu;

    @Override
    public void runOpMode() {
        initialize();
        telemetry.update();
        waitForStart();
        //imu.resetYaw();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                run();
            }
        }
    }


    public void initialize() {
        /*stft = hardwareMap.get(DcMotorEx.class, "stft");///miscare
        drft = hardwareMap.get(DcMotorEx.class, "drft");
        stspa = hardwareMap.get(DcMotorEx.class, "stspa");
        drspa = hardwareMap.get(DcMotorEx.class, "drspa");
        tun = hardwareMap.get(DcMotorEx.class, "tun");

        stspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/


        /*stspa.setDirection(DcMotor.Direction.REVERSE);
        stft.setDirection(DcMotor.Direction.REVERSE);*/

        /*imu = hardwareMap.get(IMU.class, "imu_expansion");///gyro
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN))

        );*/
        tun = hardwareMap.get(DcMotorEx.class, "tun");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
    }

    public void drive(double speed, double turn, double strafe, double maxspeed) {
        double stftpwr, drftpwr, stspapwr, drspapwr;
        stftpwr = speed + turn + strafe;
        drftpwr = speed - turn - strafe;
        stspapwr = speed + turn - strafe;
        drspapwr = speed - turn + strafe;
        /*stft.setPower(stftpwr * maxspeed);
        drft.setPower(drftpwr * maxspeed);
        stspa.setPower(stspapwr * maxspeed);
        drspa.setPower(drspapwr * maxspeed);*/
    }

    final double maxspeed = 1;
    boolean pressedb;
    boolean starerun = false;
    double speed, turn, strafe;

    public void run() {
        boolean isTriggerPressedb = gamepad1.circle;
        if (isTriggerPressedb && !pressedb) { //circle
            starerun = !starerun;
            if (starerun) {
                gamepad1.rumble(600);
            }
        }
        pressedb = isTriggerPressedb;


        speed = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        tun.setPower(gamepad2.right_stick_y);
        intake1.setPower(gamepad2.left_stick_y);
        intake2.setPower(gamepad2.left_stick_y);

        /*telemetry.addData("raw unghi", imu.getRobotYawPitchRollAngles());
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (starerun) { ///toggle field centric drive

            double tempStrafe = strafe * Math.cos(heading) + speed * Math.sin(heading);
            double tempSpeed = -strafe * Math.sin(heading) + speed * Math.cos(heading);

            speed = tempSpeed;
            strafe = tempStrafe;
            telemetry.addData("gyro", starerun);
        }
        if (gamepad1.right_trigger > 0.2) {
            drive(speed, turn, strafe, maxspeed + 0.3);
        } else if (gamepad1.left_trigger < 0.2)
            drive(speed, turn, strafe, maxspeed);*/

        /*if (gamepad1.triangle) {
            imu.resetYaw();
        }*/

    }
}
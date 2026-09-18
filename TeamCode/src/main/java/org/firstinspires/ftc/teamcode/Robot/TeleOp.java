package org.firstinspires.ftc.teamcode.Robot;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Tun;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp-test")
public class TeleOp extends LinearOpMode {
    MecanumDrive mecanumDrive;
    Intake intake;
    Tun tun;

    Follower follower;
    public static Pose redfar_pose = new Pose(80.58, 9.37, Math.toRadians(90));

    Gyro gyro;
    GoBildaPinpointDriver pinpoint;
    @Override

    public void runOpMode(){
        initi();
        telemetry.addLine("initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            run();
        }
    }
    public void initi(){
        follower = Constants.create(hardwareMap);
        follower.setPose(redfar_pose);


        tun =new Tun();
        tun.initi(hardwareMap);


        mecanumDrive = new MecanumDrive();
        mecanumDrive.initi(hardwareMap);

        intake = new Intake();
        intake.initi(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        gyro = new Gyro(hardwareMap);


    }

    public void run() {

        mecanumDrive.drive(true, gamepad1);
        intake.run(gamepad1);
        tun.run(gamepad1, gamepad2);

        follower.update();

        Pose pose = follower.pose();

        if(gamepad1.square){
            tun.tun_servo.setPosition(0.5);
        }
        if(gamepad1.circle){
            tun.tun_servo.setPosition(-0.5);
        }
        //tun.allignTun(pose.heading(), pose.x(), pose.y(), telemetry);
        telemetry.addData("x",pose.x());
        telemetry.addData("y",pose.y());
        telemetry.addData("heading", Math.toDegrees(pose.heading()));
        telemetry.addData("angle", Math.toDegrees(tun.getRotation()/tun.gear_ratio));
        telemetry.addData("Lansare", tun.getLansare());
        telemetry.addData("Gyro", Math.toDegrees(gyro.getHeading()));
        telemetry.addData("ServoTun: ", tun.getServo());
        telemetry.addData("transfer1: ", tun.transfer1.getPosition());
        telemetry.addData("transfer2: ", tun.transfer2.getPosition());
        telemetry.update();
    }

}
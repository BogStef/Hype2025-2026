package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Tun;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp-test")
public class TeleOp extends LinearOpMode {
    MecanumDrive mecanumDrive;
    Intake intake;
    Intake intake_motor;
    Tun tun;
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

        tun =new Tun();
        tun.initi(hardwareMap,1);

        mecanumDrive = new MecanumDrive();
        mecanumDrive.initi(hardwareMap);

        intake = new Intake();
        intake.initi(hardwareMap,false);

        intake_motor = new Intake();
        intake_motor.initi(hardwareMap,true);

    }

    public void run(){
        mecanumDrive.drive(false,gamepad1);//basic
        intake.run(gamepad1); //triangle
        intake_motor.run(gamepad1);//circle
        tun.run(gamepad1);//square
        telemetry.addData("intake", intake.getToggle());
        telemetry.addData("intake motor",intake_motor.getToggle());
        telemetry.addData("tun",tun.getToggle());
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp-test")
public class TeleOp extends LinearOpMode {
    MecanumDrive mecanumDrive;
    Intake intake;
    @Override
    public void runOpMode(){
        initi();
        telemetry.addLine("initialized");
        waitForStart();
        telemetry.update();
        while(opModeIsActive()){
            run();
        }
    }
    public void initi(){
        mecanumDrive = new MecanumDrive();
        mecanumDrive.initi(hardwareMap);

        intake = new Intake();
        intake.initi(hardwareMap,1);

    }
    public void run(){
        mecanumDrive.drive(false,gamepad1);//basic
        intake.run(gamepad1); //triangle
        telemetry.addData("intake", intake.getPower());
        telemetry.update();
    }
}
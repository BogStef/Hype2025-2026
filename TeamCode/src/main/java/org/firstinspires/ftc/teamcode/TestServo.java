package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp (name="TestServo")
public class TestServo extends LinearOpMode{

    public Servo hand1;
    public Servo hand2;
    public boolean hand1_toggle=false;
    public boolean hand2_toggle=false;
    @Override
    public void runOpMode()
    {
        initialize();
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while(opModeIsActive())
            {
                // TODO BUG Press dpad_down on controller before dpad_up for the first time
                run();
            }
        }
    }
    public void initialize(){
        hand1 = hardwareMap.get(Servo.class, "hand1");
        hand2 = hardwareMap.get(Servo.class, "hand2");
        hand2.setPosition(0);
        hand1.setPosition(0);

    }

    public void run(){
        if(gamepad1.circle){hand1_toggle=!hand1_toggle;}
        if(gamepad1.square){hand2_toggle=!hand2_toggle;}

        if(hand1_toggle){
            hand1.setPosition(0.2);
        }
        else{
            hand1.setPosition(0);
        }



        if(hand2_toggle){
            hand2.setPosition(0.2);
        }
        else{
            hand2.setPosition(0);
        }

    }

}

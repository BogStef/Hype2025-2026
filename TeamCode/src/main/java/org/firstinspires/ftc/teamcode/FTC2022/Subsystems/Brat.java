package org.firstinspires.ftc.teamcode.FTC2022.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Brat {
    private Servo servoBrat;
    private DcMotor motorBrat;

    private double servoPos = 0.2; // TODO
    private double servoInitPos = 0;

    // TODO variabile pt dcmotor

    public void init(HardwareMap hardwareMap){
        servoBrat = hardwareMap.get(Servo.class, "ServoBrat");
        motorBrat = hardwareMap.get(DcMotor.class, "MotorBrat");

        servoBrat.setDirection(Servo.Direction.FORWARD);
        servoBrat.setPosition(servoInitPos);

        motorBrat.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBrat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void run(Gamepad gamepad){
        if(gamepad.squareWasPressed())
            servoBrat.setPosition(servoPos);
        else
            servoBrat.setPosition(servoInitPos);

        // TODO: Dati-va seama ce plm facem cu dc motor-ul :)
    }
}

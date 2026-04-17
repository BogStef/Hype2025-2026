+=package org.firstinspires.ftc.teamcode.FTC2022.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Cleste {

    private Servo cleste;
    private double position = 0.2;
    private double initPos=0;

    public Cleste(HardwareMap hardwareMap) {
        cleste = hardwareMap.get(Servo.class,"intake");
        cleste.setDirection(Servo.Direction.FORWARD);
        cleste.setPosition(initPos);
    }

    public void run(Gamepad gamepad) {
        if (gamepad.xWasPressed()) {
            cleste.setPosition(position);
        }
        else cleste.setPosition(initPos);
    }
}

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServo intake;
    private double power;
    public boolean toggle;
    public void initi(HardwareMap hardwareMap,double p){
        intake = hardwareMap.get(CRServo.class,"intake");
        intake.setDirection(CRServo.Direction.FORWARD); ///todo
        toggle =false;
        power=p;

    }
    public void run(Gamepad gamepad){
        if(gamepad.triangleWasPressed()){
            toggle=!toggle;
        }
        if(toggle){
            intake.setPower(power);
        }
        else
            intake.setPower(0);
    }
    public double getPower(){
        return power;
    }

}

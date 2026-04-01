package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Tun {
    private DcMotorEx tun;
    private double power;
    public boolean toggle;
    public void initi(HardwareMap hardwareMap,double p){
        tun = hardwareMap.get(DcMotorEx.class,"tun");
        tun.setDirection(DcMotorEx.Direction.REVERSE); ///todo
        toggle =false;
        power=p;

    }
    public void run(Gamepad gamepad){
        if(gamepad.squareWasPressed()){
            toggle=!toggle;
        }
        if(toggle){
            tun.setPower(power);
        }
        else
            tun.setPower(0);
    }
    public double getPower(){
        return power;
    }
    public boolean getToggle(){
        return toggle;
    }

}

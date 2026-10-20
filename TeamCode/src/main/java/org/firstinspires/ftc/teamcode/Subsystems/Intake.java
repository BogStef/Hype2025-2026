package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;
    private DcMotor transfer;
    private double power;
    private boolean use_motor; 
    private boolean last_left_trigger_state = false;
    public boolean toggle;

    public void init(HardwareMap hardwareMap){
        //this.useMotor = useMotor;
        intake = hardware_map.get(DcMotor.class,"intake");
        transfer = hardware_map.get(DcMotor.class,"transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toggle = false;
    }

    public void run(Gamepad gamepad){
        if(gamepad.left_trigger>0.3){
            intake.setPower(1);
            transfer.setPower(1);
        }
        else{
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    public boolean getToggle(){
        return toggle;
    }

    public void setToggle(boolean state){
        toggle = state;
    }
}
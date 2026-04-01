package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServo intake;
    private DcMotorEx intake_motor;
    private double velocity;

    private double power;
    private boolean useMotor; // renamed for clarity
    public boolean toggle;

    public void initi(HardwareMap hardwareMap, boolean useMotor){
        this.useMotor = useMotor;

        if(useMotor){
            velocity = 6000;
            intake_motor = hardwareMap.get(DcMotorEx.class,"intake_motor");
            intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            intake_motor.setVelocityPIDFCoefficients(6.7,0,0.1,13.7);
        }
        else{
            power = 1;
            intake = hardwareMap.get(CRServo.class,"intake");
            intake.setDirection(CRServo.Direction.FORWARD);
        }

        toggle = false;
    }

    public void run(Gamepad gamepad){
        if(useMotor){
            if(gamepad.triangleWasPressed()){
                toggle = !toggle;
            }
        }
        else{
            if(gamepad.circleWasPressed()){
                toggle=!toggle;
            }
        }


        if(toggle){
            if(useMotor){
                intake_motor.setVelocity(velocity);
            }
            else{
                intake.setPower(power);
            }
        }
        else{
            if(useMotor){
                intake_motor.setVelocity(0);
            }
            else{
                intake.setPower(0);
            }
        }
    }

    public boolean getToggle(){
        return toggle;
    }

    public void setToggle(boolean state){
        toggle = state;
    }
}
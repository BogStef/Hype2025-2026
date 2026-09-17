package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
///limita dreaplta 910/gear ratio adica aproximativ 140 grade la stanga 880/gear ratio adica aprox 135
public class Tun {
    private DcMotorEx rotatie_tun;


    public Servo tun_servo;
    public Servo transfer1,transfer2;//dreapta, stanga
    private DcMotorEx lansare_tun;
    private double power_rotatie = 0.2;

    public double gear_ratio = 6.07;
    private int max_stanga = 880;//pozitiv
    private int max_dreapta = 880;//negativ

    private double pl, il, dl, fl;
    private double pr, ir, dr, fr;

    private boolean lastRightTriggerState = false;

    private double servo_tun_pos=0;
    private int lansare_velocity=1000;
    private double rest_pos_transfer2 = 0.4;
    private double rest_pos_transfer1 = 0.2;
    private double block_pos_transfer1 = 0;
    private double block_pos_transfer2 = 0.54;


    public void initi(HardwareMap hardwareMap) {
        rotatie_tun = hardwareMap.get(DcMotorEx.class, "rotatie_tun");
        lansare_tun = hardwareMap.get(DcMotorEx.class, "lansare_tun");
         lansare_tun.setDirection(DcMotorEx.Direction.FORWARD);
         lansare_tun.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

         tun_servo = hardwareMap.get(Servo.class, "tun_servo");
         tun_servo.setDirection(Servo.Direction.REVERSE);
         tun_servo.setPosition(0);

         transfer1 = hardwareMap.get(Servo.class, "transfer1");
         transfer2 = hardwareMap.get(Servo.class,"transfer2");
         transfer2.setDirection(Servo.Direction.REVERSE);
         transfer1.setDirection(Servo.Direction.FORWARD);
         transfer2.setPosition(rest_pos_transfer2);


        // FLOAT allows the motor to spin freely when power is 0
        rotatie_tun.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        rotatie_tun.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Use RUN_WITHOUT_ENCODER for the rotation motor so it doesn't
        // try to use internal PID while you are moving it by hand.
        rotatie_tun.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lansare_tun.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pl = 43.18;
        il = 1.1;
        dl = 0.2;
        fl = 13.5;

        pr = 0;
        ir = 0;
        dr = 0;
        fr = 0;
        lansare_tun.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pl, il, dl, fl));
    }

    private int target = 100;

    public void run(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad.dpad_right){
            //servo_tun_pos+=0.005;
            rotatie_tun.setPower(-0.4);
        }

        if(gamepad.dpad_left){
            //servo_tun_pos-=0.005;
            rotatie_tun.setPower(0.4);
        }
        if(gamepad.dpad_up){
            lansare_velocity+=100;
        }
        if(gamepad.dpad_down){
            lansare_velocity-=100;
        }
        if(gamepad.right_bumper){

        }
        servo_tun_pos=Math.max(0.0, Math.min(servo_tun_pos, 1.0));
        tun_servo.setPosition(servo_tun_pos);

        if(gamepad.right_trigger>0.3){
            lansare_tun.setVelocity(lansare_velocity);
            transfer2.setPosition(rest_pos_transfer2);
            transfer1.setPosition(rest_pos_transfer1);
        }
        else{
            lansare_tun.setVelocity(0);
            transfer1.setPosition(block_pos_transfer1);
            transfer2.setPosition(block_pos_transfer2);
        }
        rotatie_tun.setPower(gamepad2.right_stick_x * 0.1);

    }

    public int getLansare(){
        return lansare_velocity;
    }

    public double getServo(){
        return servo_tun_pos;

    }

    public int getRotation() {
        return rotatie_tun.getCurrentPosition();
    }

    private double length = 144;
    public void allignTun(double robot_heading, double robot_x, double robot_y){ // (0,0) stanga jos, (144, 144) red
        double raw_angle = Math.atan((length - robot_x) / (length - robot_y));
        double true_angle = raw_angle + robot_heading;
        double rotatie = rotatie_tun.getCurrentPosition()/gear_ratio;
    }
    public void limitCheck() { //TODO add so it stops it from trying to go that way
        double rotatie = rotatie_tun.getCurrentPosition();
        if (rotatie>=max_stanga)
            rotatie_tun.setPower(-0.5);
        else if (rotatie<=max_dreapta * -1)
            rotatie_tun.setPower(0.5);
    }
}
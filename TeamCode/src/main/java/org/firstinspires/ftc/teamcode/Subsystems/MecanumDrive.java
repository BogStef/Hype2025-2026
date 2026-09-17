package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive{

    private DcMotor stft,drft,drspa,stspa;
    private Gyro gyro;
    private double speed,turn,strafe,heading,maxspeed;
    
    public void initi(HardwareMap hardwareMap){
        stft = hardwareMap.get(DcMotorEx.class, "stft");///miscare
        drft = hardwareMap.get(DcMotorEx.class, "drft");
        stspa = hardwareMap.get(DcMotorEx.class, "stspa");
        drspa = hardwareMap.get(DcMotorEx.class, "drspa");

        stft.setDirection(DcMotorSimple.Direction.FORWARD);
        drft.setDirection(DcMotorSimple.Direction.REVERSE);///todo
        drspa.setDirection(DcMotorSimple.Direction.REVERSE);
        stspa.setDirection(DcMotorSimple.Direction.FORWARD);

        stft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gyro = new Gyro(hardwareMap);
        maxspeed = 1;

    }
    public void drive(boolean field,Gamepad gamepad){
        update_move(gamepad);
        if(!field){
            robot_drive(speed,turn,strafe,maxspeed);
        }
        else{
            field_drive(speed,turn,strafe,maxspeed,heading);
        }

    }

    private void update_move(Gamepad gamepad){
        speed = -gamepad.left_stick_y;
        turn = -gamepad.right_stick_x;
        strafe = gamepad.left_stick_x;
        heading = gyro.getHeading();
    }

    private void robot_drive(double speed, double turn, double strafe, double maxspeed) {

        double stftpwr = speed + turn + strafe;
        double drftpwr = speed - turn - strafe;
        double stspapwr = speed + turn - strafe;
        double drspapwr = speed - turn + strafe;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(stftpwr),
                        Math.max(
                                Math.abs(drftpwr),
                                Math.max(Math.abs(stspapwr), Math.abs(drspapwr))
                        )
                )
        );

        stftpwr /= max;
        drftpwr /= max;
        stspapwr /= max;
        drspapwr /= max;

        stft.setPower(stftpwr * maxspeed);
        drft.setPower(drftpwr * maxspeed);
        stspa.setPower(stspapwr * maxspeed);
        drspa.setPower(drspapwr * maxspeed);
    }

    private void field_drive(double speed,double turn,double strafe,double maxspeed,double heading){
        double stftpwr,drftpwr,stspapwr,drspapwr;
        double temp_strafe = strafe * Math.cos(heading) + speed * Math.sin(heading);
        double temp_speed = -strafe * Math.sin(heading) + speed * Math.cos(heading);

        strafe = temp_strafe;
        speed = temp_speed;

        stftpwr = speed + turn + strafe;
        drftpwr = speed - turn - strafe;
        stspapwr = speed + turn - strafe;
        drspapwr = speed - turn + strafe;
        stft.setPower(stftpwr*maxspeed);
        drft.setPower(drftpwr*maxspeed);
         stspa.setPower(stspapwr*maxspeed);
         drspa.setPower(drspapwr*maxspeed);
    }

    public void testStrafe() {
        stft.setPower(1);
        drft.setPower(-1);
        stspa.setPower(-1);
        drspa.setPower(1);
    }





}

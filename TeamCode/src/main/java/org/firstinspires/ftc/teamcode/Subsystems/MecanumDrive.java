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

        stft.setDirection(DcMotorSimple.Direction.REVERSE);
        drft.setDirection(DcMotorSimple.Direction.FORWARD);///todo
        drspa.setDirection(DcMotorSimple.Direction.FORWARD);
        stspa.setDirection(DcMotorSimple.Direction.REVERSE);
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
        turn = gamepad.right_stick_x;
        strafe = gamepad.left_stick_x;
        heading = gyro.getHeading();
    }

    private void robot_drive(double speed,double turn,double strafe,double maxspeed){
            double stftpwr,drftpwr,stspapwr,drspapwr;
            stftpwr = speed + turn + strafe;
            drftpwr = speed - turn - strafe;
            stspapwr = speed + turn - strafe;
            drspapwr = speed - turn + strafe;
            stft.setPower(stftpwr*maxspeed);
            drft.setPower(drftpwr*maxspeed);
            stspa.setPower(stspapwr*maxspeed);
            drspa.setPower(drspapwr*maxspeed);
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




}

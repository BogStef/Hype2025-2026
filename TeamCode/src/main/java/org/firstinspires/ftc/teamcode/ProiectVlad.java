package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="autofata")
public class ProiectVlad extends LinearOpMode {
    private DcMotor stft,stspa,drft,drspa;

    ElapsedTime timer = new ElapsedTime();
    public double timp;
    boolean timer_reseted = false;


    @Override
    public void runOpMode() throws InterruptedException {
        initi();
        waitForStart();
        timp = timer.seconds();
        if(opModeIsActive())
        {
            while(timer.seconds() - timp < 0.5){
                stft.setPower(1);
                drft.setPower(1);
                stspa.setPower(1);
                drspa.setPower(1);
            }
            timp = timer.seconds();
            while(timer.seconds() - timp < 2){
                stft.setPower(0);
                drft.setPower(0);
                stspa.setPower(0);
                drspa.setPower(0);
            }
        }
    }

    public void initi() {
        stft = hardwareMap.get(DcMotor.class, "stft");
        drft = hardwareMap.get(DcMotor.class,"drft");
        stspa=hardwareMap.get(DcMotor.class,"stspa");
        drspa = hardwareMap.get(DcMotor.class,"drspa");

        stspa.setDirection(DcMotor.Direction.REVERSE);
        stft.setDirection(DcMotor.Direction.REVERSE);

        stft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

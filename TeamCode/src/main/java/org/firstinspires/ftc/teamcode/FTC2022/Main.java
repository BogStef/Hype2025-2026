package org.firstinspires.ftc.teamcode.FTC2022;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FTC-2022")
public class Main extends LinearOpMode {

    @Override
    public void runOpMode(){
        initi();
        telemetry.addLine("initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            run();
        }
    }
    public void initi(){

    }

    public void run(){

    }
}
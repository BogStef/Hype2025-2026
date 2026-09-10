package org.firstinspires.ftc.teamcode.FTC2022;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTC2022.Subsystems.Cleste;
import org.firstinspires.ftc.teamcode.FTC2022.Subsystems.Brat;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FTC-2022")
public class Main extends LinearOpMode {
    Cleste cleste;
    Brat brat;

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

        cleste = new Cleste();
        cleste.init(hardwareMap);

        brat = new Brat(); // TODO: nu schimbam init cu un constructor?
        brat.init(hardwareMap);
    }

    public void run(){
        cleste.run(gamepad1);
    }
}
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test")
public class Test extends LinearOpMode {
    public DcMotorEx motor;

    @Override
    public void runOpMode() {
        initi();
        telemetry.addLine("initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            run();
        }
    }

    public void initi() {
        
    }

    public void run() {

    }
}


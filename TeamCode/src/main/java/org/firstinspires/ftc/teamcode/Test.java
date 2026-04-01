package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test")
public class Test extends LinearOpMode {
    public CRServo pultetrix;

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
        pultetrix = hardwareMap.get(CRServo.class, "pultetrix");
        pultetrix.setDirection(CRServo.Direction.FORWARD);
    }

    public void run() {
        pultetrix.setPower(1);
    }
}


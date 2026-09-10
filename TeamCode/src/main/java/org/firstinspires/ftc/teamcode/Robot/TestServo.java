package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test")
public class TestServo extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.get(Servo.class, "tun_servo");

        telemetry.addLine("Servo initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.circle) {
                servo.setPosition(0.0);
            }

            if (gamepad1.cross) {
                servo.setPosition(0.5);
            }

            if (gamepad1.square) {
                servo.setPosition(1.0);
            }

            telemetry.addData("Position", "A=0 / B=0.5 / X=1");
            telemetry.update();
        }
    }
}

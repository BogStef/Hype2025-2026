package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TelAviv")
public class TelAviv extends LinearOpMode {

    @Override
    public void runOpMode() {


        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(1);

        }
    }
}

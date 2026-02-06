package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="test")
public class ProiectVlad extends OpMode {
    private DcMotor motor;
    private DcMotor motor2;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}

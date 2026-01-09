package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ProiectVlad_Power")
public class ProiectVlad extends OpMode {
    private DcMotor motor;
    private DcMotor motor2;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        // Motor stops instead of coasting when power is zero
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // RUN_WITHOUT_ENCODER disables velocity control
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        // Motor stops instead of coasting when power is zero
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // RUN_WITHOUT_ENCODER disables velocity control
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    }

    @Override
    public void loop() {
        // Adjust this value to make motor spin slowly
        // 0.1 = 10% power, 0.2 = 20%, etc.
        motor.setPower(-0.33); // ~33% power
        motor2.setPower(-0.33
        );
    }
}

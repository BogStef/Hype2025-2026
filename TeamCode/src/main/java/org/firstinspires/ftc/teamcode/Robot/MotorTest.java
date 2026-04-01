package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test Motor - prezentare")
public class MotorTest extends LinearOpMode {
    DcMotor motor;
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
        motor = hardwareMap.get(DcMotor.class,"motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void run(){

        if(gamepad1.cross){
            motor.setPower(1);
        }
        else motor.setPower(0);

        telemetry.update();
    }
}
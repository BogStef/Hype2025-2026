package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        initialize();
        telemetry.update();
        waitForStart();
            while (opModeIsActive()) {
                run();
            }

    }
        public DistanceSensor rangeSensor;
    public void initialize(){
            rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");

    }
    public void run(){
        telemetry.update();
        telemetry.addData("distanta: >",rangeSensor.getDistance(DistanceUnit.CM));

    }
}

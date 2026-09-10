package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp-Thomas")
public class TeleOpThomas extends OpMode {

    public DcMotorEx lansare_tun;

    public double highVelocity = 2000;
    public double lowVelocity = 1000;
    public double curTarget = highVelocity;

    Intake intake;

    // PIDF coefficients
    double p = 0;
    double i = 0;
    double d = 0;
    double f = 0;

    // Which coefficient is currently selected:
    // 0 = P, 1 = I, 2 = D, 3 = F
    int selectedParam = 0;

    final String[] paramNames = {"P", "I", "D", "F"};

    // Step sizes for PIDF adjustment
    final double[] stepSizes = {
            10.0,
            1.0,
            0.1,
            0.01,
            0.001,
            0.0001
    };

    int stepIndex = 0;

    @Override
    public void init() {

        // --- Motor setup ---
        lansare_tun = hardwareMap.get(DcMotorEx.class, "lansare_tun");

        lansare_tun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Kept from your FIRST variant
        lansare_tun.setDirection(DcMotorSimple.Direction.FORWARD);

        // Apply initial PIDF values
        applyCoefficients();

        // --- Intake setup ---
        intake = new Intake();
        intake.initi(hardwareMap);

        telemetry.addLine("Init complete.");
        telemetry.addLine("PIDF Tuner ready.");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ==========================================
        // INTAKE
        // ==========================================

        intake.run(gamepad1);


        // ==========================================
        // VELOCITY CONTROL
        // ==========================================

        // Y: Toggle between high and low velocity
        if (gamepad1.yWasPressed()) {
            if (curTarget == highVelocity) {
                curTarget = lowVelocity;
            } else {
                curTarget = highVelocity;
            }
        }


        // ==========================================
        // PIDF TUNER CONTROLS
        // ==========================================

        // B: Cycle through step sizes
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // A: Cycle selected parameter
        // P -> I -> D -> F -> P
        if (gamepad1.aWasPressed()) {
            selectedParam = (selectedParam + 1) % 4;
        }

        double step = stepSizes[stepIndex];

        // D-Pad Up: Increase selected parameter
        if (gamepad1.dpadUpWasPressed()) {
            adjustParam(selectedParam, step);
        }

        // D-Pad Down: Decrease selected parameter
        if (gamepad1.dpadDownWasPressed()) {
            adjustParam(selectedParam, -step);
        }


        // ==========================================
        // APPLY PIDF
        // ==========================================

        applyCoefficients();


        // ==========================================
        // RUN MOTOR
        // ==========================================

        lansare_tun.setVelocity(curTarget);


        // ==========================================
        // TELEMETRY
        // ==========================================

        double curVelocity = lansare_tun.getVelocity();
        double error = curTarget - curVelocity;

        telemetry.addLine("========= PIDF TUNER =========");

        telemetry.addData(
                "Target Velocity",
                "%.0f ticks/s",
                curTarget
        );

        telemetry.addData(
                "Current Velocity",
                "%.0f ticks/s",
                curVelocity
        );

        telemetry.addData(
                "Error",
                "%.2f ticks/s",
                error
        );

        telemetry.addLine("-----------------------------");

        telemetry.addData(
                "Step Size",
                "%.5f  (B to change)",
                stepSizes[stepIndex]
        );

        telemetry.addData(
                "Active Parameter",
                "%s  (A to change)",
                paramNames[selectedParam]
        );

        telemetry.addLine("-----------------------------");

        telemetry.addData(
                "[P]",
                "%.5f%s",
                p,
                selectedParam == 0 ? " <<" : ""
        );

        telemetry.addData(
                "[I]",
                "%.5f%s",
                i,
                selectedParam == 1 ? " <<" : ""
        );

        telemetry.addData(
                "[D]",
                "%.5f%s",
                d,
                selectedParam == 2 ? " <<" : ""
        );

        telemetry.addData(
                "[F]",
                "%.5f%s",
                f,
                selectedParam == 3 ? " <<" : ""
        );

        telemetry.addLine("-----------------------------");

        telemetry.addLine("A: Select P/I/D/F");
        telemetry.addLine("B: Change Step Size");
        telemetry.addLine("D-Pad Up/Down: Adjust");
        telemetry.addLine("Y: Toggle Speed");

        telemetry.update();
    }


    // ==========================================
    // ADJUST SELECTED PIDF PARAMETER
    // ==========================================

    private void adjustParam(int param, double delta) {

        switch (param) {

            case 0:
                p += delta;
                break;

            case 1:
                i += delta;
                break;

            case 2:
                d += delta;
                break;

            case 3:
                f += delta;
                break;
        }
    }


    // ==========================================
    // APPLY PIDF COEFFICIENTS TO MOTOR
    // ==========================================

    private void applyCoefficients() {

        PIDFCoefficients pidfCoefficients =
                new PIDFCoefficients(p, i, d, f);

        lansare_tun.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );
    }
}

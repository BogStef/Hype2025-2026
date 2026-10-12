package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

///
///
///
/// ///limita dreaplta 910/gear ratio adica aproximativ 140 grade la stanga 880/gear ratio adica aprox 135
public class Tun {
    private DcMotorEx rotatie_tun;
    private float hight=0;

    public Servo tun_servo;
    public Servo transfer1,transfer2;//dreapta, stanga
    private DcMotorEx lansare_tun;
    private double power_rotatie = 0.2;

    public double gear_ratio = 6.07;
    private int max_stanga = 135;//pozitiv
    private int max_dreapta = 150;//negativ

    private double pl, il, dl, fl;
    private double pr, ir, dr, fr;
    private double lastError = 0;
    private com.qualcomm.robotcore.util.ElapsedTime pidTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    public static double kp = 0.022;
    public static double ki = 0.0;
    public static double kd = 0.0015;
    private boolean lastRightTriggerState = false;

    private double servo_tun_pos=0;
    private int lansare_velocity=1000;
    private double rest_pos_transfer2 = 0.4;
    private double rest_pos_transfer1 = 0.2;
    private double block_pos_transfer1 = 0;
    private double block_pos_transfer2 = 0.54;


    public void initi(HardwareMap hardwareMap) {
        rotatie_tun = hardwareMap.get(DcMotorEx.class, "rotatie_tun");
        lansare_tun = hardwareMap.get(DcMotorEx.class, "lansare_tun");
         lansare_tun.setDirection(DcMotorEx.Direction.FORWARD);
         lansare_tun.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

         tun_servo = hardwareMap.get(Servo.class, "tun_servo");
         tun_servo.setDirection(Servo.Direction.REVERSE);
         tun_servo.setPosition(0);

         transfer1 = hardwareMap.get(Servo.class, "transfer1");
         transfer2 = hardwareMap.get(Servo.class,"transfer2");
         transfer2.setDirection(Servo.Direction.REVERSE);
         transfer1.setDirection(Servo.Direction.FORWARD);
         transfer2.setPosition(rest_pos_transfer2);


        // FLOAT allows the motor to spin freely when power is 0
        rotatie_tun.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //rotatie_tun.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotatie_tun.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Use RUN_WITHOUT_ENCODER for the rotation motor so it doesn't
        // try to use internal PID while you are moving it by hand.
        rotatie_tun.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lansare_tun.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pl = 43.18;
        il = 1.1;
        dl = 0.2;
        fl = 13.5;

        pr = 0;
        ir = 0;
        dr = 0;
        fr = 0;
        lansare_tun.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pl, il, dl, fl));
    }


    public void run(Gamepad gamepad, Gamepad gamepad2) {

        if(gamepad.dpad_right){
            servo_tun_pos+=0.005;
        }

        if(gamepad.dpad_left){
            servo_tun_pos-=0.005;
        }
        if(gamepad.dpad_up){
            lansare_velocity+=100;
        }
        if(gamepad.dpad_down){
            lansare_velocity-=100;
        }
        if(gamepad.right_bumper){

        }
        if (gamepad2.right_trigger>0)
            //rotatie_tun.setPower(0);
            servo_tun_pos=Math.max(0.0, Math.min(servo_tun_pos, 1.0));
        tun_servo.setPosition(servo_tun_pos);

        if(gamepad.right_trigger>0.3){
            lansare_tun.setVelocity(lansare_velocity);
            transfer2.setPosition(rest_pos_transfer2);
            transfer1.setPosition(rest_pos_transfer1);
        }
        else{
            lansare_tun.setVelocity(0);
            transfer1.setPosition(block_pos_transfer1);
            transfer2.setPosition(block_pos_transfer2);
        }


    }

    public int getLansare(){
        return lansare_velocity;
    }

    public double getServo(){
        return servo_tun_pos;

    }

    public int getRotation() {
        return rotatie_tun.getCurrentPosition();
    }

    public double target_x = 144.0;
    public double target_y = 144.0;
    public double ticks_per_degree = 1.0; // Adjust if motor encoder ticks differ from 1 tick per degree after gear ratio

    /**
     * Auto-aligns the turret horizontally toward specified target field coordinates.
     *
     * @param robot_heading Current robot heading in radians
     * @param robot_x       Current robot X position on field
     * @param robot_y       Current robot Y position on field
     * @param target_x      Target X coordinate on field
     * @param target_y      Target Y coordinate on field
     * @param telemetry     Telemetry handle for driver debugging
     */
    public void allign_tun_horizontal(double robot_heading, double robot_x, double robot_y, double target_x, double target_y, Telemetry telemetry) {
        double delta_x = target_x - robot_x;
        double delta_y = target_y - robot_y;
        double field_angle_radians = Math.atan2(delta_y, delta_x);
        double target_relative_radians = field_angle_radians - robot_heading;
        double target_angle_degrees = Math.toDegrees(target_relative_radians);

        // Convert motor encoder ticks to turret degrees
        double current_angle_degrees = (rotatie_tun.getCurrentPosition() / gear_ratio) / ticks_per_degree;

        // Normalize error to [-180, 180] degrees
        double error = target_angle_degrees - current_angle_degrees;
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        // Derivative term with safety guard against division by zero
        double seconds = pidTimer.seconds();
        double derivative = 0;
        if (seconds > 0.0001) {
            derivative = (error - lastError) / seconds;
            pidTimer.reset();
            lastError = error;
        }

        double power = (kp * error) + (kd * derivative);
        double max_power = 0.8;
        power = Math.max(-max_power, Math.min(max_power, power));

        // Deadband threshold to eliminate motor chatter near target
        if (Math.abs(error) < 0.8) {
            power = 0;
        }

        // Soft limit enforcement
        if (current_angle_degrees >= max_stanga && power > 0) {
            power = 0;
        } else if (current_angle_degrees <= (-max_dreapta) && power < 0) {
            power = 0;
        }

        if (telemetry != null) {
            telemetry.addData("Turret Target", "%.1f°", target_angle_degrees);
            telemetry.addData("Turret Current", "%.1f°", current_angle_degrees);
            telemetry.addData("Turret Error", "%.1f°", error);
            telemetry.addData("Turret Power", "%.2f", power);
        }

        rotatie_tun.setPower(power);
    }

    /**
     * Overloaded horizontal alignment defaulting to target_x and target_y.
     */
    public void allign_tun_horizontal(double robot_heading, double robot_x, double robot_y, Telemetry telemetry) {
        allign_tun_horizontal(robot_heading, robot_x, robot_y, this.target_x, this.target_y, telemetry);
    }
    public void align_tun_vertical(double robot_x, double robot_y){
    }

    /**
     * Calculates required launcher motor velocity (in encoder ticks per second)
     * based on horizontal distance and target height using projectile kinematics.
     *
     * @param distance Horizontal distance to target (in inches)
     * @param height   Target height relative to launcher pivot (in inches)
     * @param launchAngleDegrees Fixed launch angle of launcher in degrees (e.g., 35.0°)
     * @return Required motor velocity in ticks per second
     */
    public double calculateLansareVelocity(double distance, double height, double launchAngleDegrees) {
        if (distance <= 0) return 0;

        double launchAngleRad = Math.toRadians(launchAngleDegrees);
        double g = 386.09; // Gravity acceleration in inches/s^2

        double tanTheta = Math.tan(launchAngleRad);
        double cosTheta = Math.cos(launchAngleRad);

        // Denominator in projectile equation: 2 * (d * tan(theta) - h)
        double denominator = 2 * (distance * tanTheta - height);

        // Prevent division by zero or imaginary square roots
        if (denominator <= 0.01) {
            denominator = 0.01;
        }

        // Required exit velocity in inches per second
        double requiredV0 = (distance / cosTheta) * Math.sqrt(g / denominator);

        // Empirical scaling constant converting linear launch velocity (in/s) to motor ticks/s
        double ticksPerInSec = 12.5;
        double targetVelocity = requiredV0 * ticksPerInSec;

        // Clamp velocity within safe motor operational limits
        double minVelocity = 500.0;
        double maxVelocity = 2800.0;
        return Math.max(minVelocity, Math.min(maxVelocity, targetVelocity));
    }

    /**
     * Overloaded method using default 35.0° launch angle.
     *
     * @param distance Horizontal distance to target (in inches)
     * @param height   Target height relative to launcher (in inches)
     * @return Required motor velocity in ticks per second
     */
    public double calculateLansareVelocity(double distance, double height) {
        return calculateLansareVelocity(distance, height, 35.0);
    }

    /**
     * Calculates speed/velocity based on distance and height, updates lansare_velocity and runs motor.
     *
     * @param distance Horizontal distance to target
     * @param height   Target height relative to launcher
     */
    public void shoot(double distance, double height) {
        double velocity = calculateLansareVelocity(distance, height);
        this.lansare_velocity = (int) Math.round(velocity);
        lansare_tun.setVelocity(this.lansare_velocity);
    }

    public void shoot(float target_distance) {
        shoot(target_distance, this.hight);
    }
}
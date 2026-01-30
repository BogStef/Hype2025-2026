package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test")
public class Test extends LinearOpMode {

    public Servo rev;
    public boolean lastSquare = false;
    public boolean toggle_tun;
    public DcMotor tun1;

    public final double revpos1=0.22;
    public final double revpos2=0.58;
    public final double revpos3 = 0.96;
    public final double viteza_tun = 1;
    public Limelight3A limelight;
    public DcMotor tun2;
    //double servoPos = 0.0;
    ElapsedTime shootTimer = new ElapsedTime();
    public Servo push_tun;
    public boolean shooting = false;
    public double distanceCM;
    public DistanceSensor rangeSensor;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
        while (opModeIsActive()) {
            if(gamepad1.dpad_right){
                rev_right();
            }
            else if(gamepad1.dpad_left){
                rev_left();
            }
            else if(gamepad1.square){
                tun(viteza_tun);
            }
        }
    }
    public void initialize(){
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        motif_code = -1;
        rev = hardwareMap.get(Servo.class, "rev");
        telemetry.addLine("init");
        telemetry.update();
        tun1 = hardwareMap.get(DcMotor.class,"tun1");
        tun2=hardwareMap.get(DcMotor.class,"tun2");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setDirection(DcMotor.Direction.REVERSE);
        toggle_tun=false;
    }
    public void rev_left(){
        if(rev.getPosition()==revpos1)
            rev.setPosition(revpos2);
        else if(rev.getPosition()==revpos2)
            rev.setPosition(revpos3);
        else if(rev.getPosition()==revpos3)
            rev.setPosition(revpos1);
        else
            rev.setPosition(revpos1);


    }

    public void rev_right(){
        if(rev.getPosition()==revpos1)
            rev.setPosition(revpos3);
        else if(rev.getPosition()==revpos2)
            rev.setPosition(revpos1);
        else if(rev.getPosition()==revpos3)
            rev.setPosition(revpos2);
        else
            rev.setPosition(revpos1);
    }
    void tun(double vit) {
        boolean currentSquare = gamepad2.square;
        if (currentSquare && !lastSquare) {
            toggle_tun = !toggle_tun;
        }
        lastSquare = currentSquare;
        if (toggle_tun) {
            tun1.setPower(vit);
            tun2.setPower(vit);
        } else {
            tun1.setPower(0);
            tun2.setPower(0);
        }
    }
    public ColorSensor colorsensor;
    public int getColor() {
        double red, blue, green, alpha, error;
        error = 0;
        red = colorsensor.red();
        blue = colorsensor.blue();
        green = colorsensor.green();
        alpha = colorsensor.alpha();
        if ((red == 0 + error || red == 0 - error) &&
                (blue == 0 + error || blue == 0 - error) &&
                (green == 0 + error || green == 0 - error) &&
                (alpha == 0 + error || alpha == 0 - error)) {
            return 0;
        }
        if ((red == 1 + error || red == 1 - error) &&
                (blue == 1 + error || blue == 1 - error) &&
                (green == 0 + error || green == 0 - error) &&
                (alpha == 0 + error || alpha == 0 - error)) {
            return 1;
        }
        return -1;
    }
    public int nrBall = 0;
    public int nrBallBasic = 0;
    public boolean[] orderBall = {false , false , false};
    public boolean readytoshoot = false;
    public void storage_sort(){
        switch(nrBall) {
            case 0:
                if (distanceCM < 0) {//TODO valori
                    if (getColor() == neededOrder[0]) {
                        rev_right();
                        orderBall[0] = true;
                        nrBall++;
                    } else {
                        rev_left();
                        nrBall++;
                    }
                }
                break;
            case 1:
                if (distanceCM < 0) {//TODO valori
                    if (getColor() == neededOrder[1]) {
                        orderBall[1] = true;
                    }
                    if (orderBall[0]) {
                        rev_right();
                        nrBall++;
                    } else {
                        rev_left();
                        nrBall++;
                    }
                }
                break;
            case 2:
                if (distanceCM < 0) {//TODO valori
                    if (getColor() == neededOrder[2]) {
                        orderBall[2] = true;
                    }
                    if (orderBall[1] == false && orderBall[0] == false) {
                        rev_left();
                    }
                }
                readytoshoot = true;
                break;
        }
        if(readytoshoot){
            if(orderBall[1]){
                shoot();
                rev_right();
                shoot();
                rev_right();
                shoot();
            }
            else{
                shoot();
                rev_left();
                shoot();
                rev_left();
                shoot();
            }
        }
    }
    public void updateShoot() {
        if (shooting && shootTimer.milliseconds() > 300) {
            push_tun.setPosition(0.0);//TODO change value
            shooting = false;
        }
    }
    public int[] neededOrder = {0,0,0};//0 - purple, 1 - green
    public int motif_code;
    public int motif() {
        if (motif_code == 21 || motif_code == 22 || motif_code == 23) {
            return motif_code;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }
        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return -1;
        }
        for (FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (id == 21 || id == 22 || id == 23) {
                motif_code = id;       // Salvăm motiv-ul definitiv
                telemetry.addData("Motif Found", id);
                telemetry.update();
                neededOrder[id-21] = 1;
                return id;
            }
        }
        return -1;
    }
    public void shoot() {
        push_tun.setPosition(1.0);
        shootTimer.reset();
        shooting = true;
    }

}

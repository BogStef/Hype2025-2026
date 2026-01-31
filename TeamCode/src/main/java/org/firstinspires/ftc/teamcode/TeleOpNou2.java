

package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOp - BUN")

public class TeleOpNou2 extends LinearOpMode {
    public DcMotorEx drft, stft, drspa, stspa;
    public DcMotor tun1;
    public DcMotor tun2;
    public IMU imu;
    public double distanceCM;
    public ColorSensor colorsensor;
    Limelight3A limelight;
    ElapsedTime shootTimer = new ElapsedTime();

    public boolean pass_intake;
    Follower follower;
    public boolean toggle_tun;
    public DistanceSensor rangeSensor;
    boolean lastDpadUp = false;



    public DcMotor intake_up;
    public DcMotor intake_down;

    public Servo push_tun;
    public Servo rev;

    public CRServo improvizatie_intake;

    public final double revpos1=0.2;
    public final double revpos2=0.56;
    public final double revpos3 = 0.94;



    public final double vmax = 0.7;
    public int motif_code;
    public float distanta_proxsensorCM=16;//real = 13.3

    public int[] neededOrder = {0,0,0};//0 - purple, 1 - green

    int shootState=0;

    public int getColor() { /// 0==mov 1==green
        double red, blue, green, alpha, error;
        error = 120;
        red = colorsensor.red();
        blue = colorsensor.blue();
        green = colorsensor.green();
        alpha = colorsensor.alpha();
        if(green>95&&green>blue&&green>red){
            return 1;
        }
        if(red>green||blue>green)
            return 0;

        return -1;
    }

    /// TODO - colorsensor test


    @Override
    public void runOpMode() {
        initialize();
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // TODO BUG Press dpad_down on controller before dpad_up for the first time
                run();
            }
        }
    }


    public void initialize() {
        stft = hardwareMap.get(DcMotorEx.class, "stft");///miscare
        drft = hardwareMap.get(DcMotorEx.class, "drft");
        stspa = hardwareMap.get(DcMotorEx.class, "stspa");
        drspa = hardwareMap.get(DcMotorEx.class, "drspa");
        stspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stspa.setDirection(DcMotor.Direction.REVERSE);
        stft.setDirection(DcMotor.Direction.REVERSE);

        stft.setVelocityPIDFCoefficients(p, i, d, f);
        drft.setVelocityPIDFCoefficients(p, i, d, f);
        stspa.setVelocityPIDFCoefficients(p, i, d, f);
        drspa.setVelocityPIDFCoefficients(p, i, d, f);



        tun1 = hardwareMap.get(DcMotor.class, "tun1");
        tun2 = hardwareMap.get(DcMotor.class, "tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun1.setDirection(DcMotor.Direction.REVERSE);

        intake_up = hardwareMap.get(DcMotor.class, "intake_up");///intake
        intake_down = hardwareMap.get(DcMotor.class, "intake_down");
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_up.setDirection(DcMotor.Direction.REVERSE);
        intake_down.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");///gyro
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP))

        );
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");///camera
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
        limelight.stop();

        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");


        push_tun = hardwareMap.get(Servo.class,"push_tun");
        rev = hardwareMap.get(Servo.class,"rev");
        improvizatie_intake = hardwareMap.get(CRServo.class,"improvizatie_intake");

        pass_intake = false;///pass intake
        toggle_tun = false;
        motif_code = -1;
        sort=false;//daca intram in modul de sortare(la finalul meciului)

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));///todo

        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
    }

    boolean pressedl = false;
    boolean starerun = true;
    boolean pressedb = false;
    double resunghi = 0;
    double unghi = 0;
    double maxi = 0;


    double p = 10;//25
    double i = 0;
    double d = 5;//10
    double f = 1670;//36

    boolean sort;
    boolean shooting = false;
    boolean lastrighttrigger=false;

    //PID

    public void run() {
        // valorile pt senzoru de proxi
        //telemetry.addData("Distance CM", distanceCM);
        telemetry.update();//da pe driver hub

        boolean isTriggerPressed = gamepad1.cross;
        boolean isTriggerPressedb = gamepad1.circle;

        if (isTriggerPressedb && !pressedb) {
            starerun = !starerun;
        }

        if (isTriggerPressed && !pressedl) {
            resunghi = -unghi() + 180;
        }

        pressedl = isTriggerPressed;
        pressedb = isTriggerPressedb;

        double stftpwr;
        double stspapwr;
        double drftpwr;
        double drspapwr;


            double speed;
            if (gamepad2.right_trigger == 1) {
                speed = -gamepad1.left_stick_y - gamepad2.right_stick_y / 3;
            } else {
                speed = -gamepad1.left_stick_y;
            }

            final double turn = gamepad1.right_stick_x;
            final double strafe = gamepad1.left_stick_x;

            stftpwr = speed + turn + strafe;
            drftpwr = speed - turn - strafe;
            stspapwr = speed + turn - strafe;
            drspapwr = speed - turn + strafe;

            double raport =0.585; /// 0.585; //experimental
            double vit = 2787; //435 rpm / 60 * 345.5 ticks/rev

//            stft.setVelocity(stftpwr * raport * vit);
//            drft.setVelocity(drftpwr * raport * vit);
//            stspa.setVelocity(stspapwr * raport * vit);
//            drspa.setVelocity(drspapwr * raport * vit);

        stftpwr = Range.clip(stftpwr, -1, 1);
        drftpwr = Range.clip(drftpwr, -1, 1);
        stspapwr = Range.clip(stspapwr, -1, 1);
        drspapwr = Range.clip(drspapwr, -1, 1);
        double max = Math.max(Math.abs(stftpwr), Math.abs(drftpwr));
        max = Math.max(max, Math.abs(stspapwr));
        max = Math.max(max, Math.abs(drspapwr));

        if(max > 1){
            stftpwr /= max;
            drftpwr /= max;
            stspapwr /= max;
            drspapwr /= max;
        }

        double maxspeed=0.7;
        stft.setPower(stftpwr*maxspeed);
        drft.setPower(drftpwr*maxspeed);
        stspa.setPower(stspapwr*maxspeed);
        drspa.setPower(drspapwr*maxspeed);
        ////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////

        //distanceCM = rangeSensor.getDistance(DistanceUnit.CM);//update continuu la distanta
        limelight.stop();


//        if (motif_code == -1)  motif_code = motif(); //cauta motif
//        else limelight.stop();

        ///OVVERIDEURI
        force_right();///PAD URI
        force_left();


        //colorsensor.enableLed(false);

        boolean currentrighttrigger=gamepad2.right_bumper;

        if(currentrighttrigger && !lastrighttrigger){
            shooting=true;
        }
        lastrighttrigger=currentrighttrigger;

        if(shooting){
            shoot();
            if(functionshootstate == 2){
                shooting = false;
                functionshootstate = 0;
            }
        }

        tun(1);///0.367 ///square
        intake(.7); ///triunghi = on //cross = stop //circle= pass (parametru viteza de la 0 la 1)






    }

    boolean lastSquare = false;

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
    boolean lastLeftDpad = false;
    void force_left() {
        boolean currentLeftDpad=gamepad2.dpad_left;
        if(currentLeftDpad && !lastLeftDpad){
            revLeft();
        }
        lastLeftDpad = currentLeftDpad;
    }
    boolean lastRightDpad=false;
    void force_right(){
        boolean currentRightDpad=gamepad2.dpad_right;
        if(currentRightDpad && !lastRightDpad){
            revRight();
        }
        lastRightDpad = currentRightDpad;
    }




    ///triunghi = on //cross = stop //circle= pass (parametru viteza de la 0 la 1)
    void intake(double vit) {
        if (gamepad2.triangle) {
            intake_up.setPower(vit);
            intake_down.setPower(vit);
            improvizatie_intake.setPower(.6);
        }
        if (gamepad2.cross) {
            intake_up.setPower(0);
            intake_down.setPower(0);
            improvizatie_intake.setPower(0);
        }

        if (gamepad2.circle) {
            intake_up.setPower(0);
            pass_intake = !pass_intake;
            if (pass_intake) {
                intake_down.setPower(vit);
            } else {
                intake_down.setPower(-1*vit);
                intake_up.setPower(-vit);
            }
            improvizatie_intake.setPower(0);
        }
    }

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
                /*switch(id){
                    case 21:
                        neededOrder[0] = 1;
                        neededOrder[1] = 0;
                        neededOrder[2] = 0;
                        break;
                    case 22:
                        neededOrder[0] = 0;
                        neededOrder[1] = 1;
                        neededOrder[2] = 0;
                        break;
                    case 23:
                        neededOrder[0] = 0;
                        neededOrder[1] = 0;
                        neededOrder[2] = 1;
                        break;
                }*/
                return id;


            }
        }

        return -1;
    }

    int caterun = 0;

    ///nico stie
    public double unghi() {


        double unghi = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //telemetry.addData(" ", unghi);
        //telemetry.update();
        return unghi + 180;
    }

    public void rotation() {
        final double rot = gamepad1.right_stick_x;

        drft.setVelocity(2897 * -vmax * rot);
        stft.setVelocity(2897 * vmax * rot);
        drspa.setVelocity(2897 * -vmax * rot);
        stspa.setVelocity(2897 * vmax * rot);
    }
    public void revRight() {
        double pos = rev.getPosition();
        double error = 0.01;

        if (Math.abs(pos - revpos1) < error) {
            rev.setPosition(revpos3);
        } else if (Math.abs(pos - revpos2) < error) {
            rev.setPosition(revpos1);
        } else if (Math.abs(pos - revpos3) < error) {
            rev.setPosition(revpos2);
        } else {
            rev.setPosition(revpos1);
        }
    }

    public void revLeft() {
        double pos = rev.getPosition();
        double error = 0.01;

        if (Math.abs(pos - revpos1) < error) {
            rev.setPosition(revpos2);
        } else if (Math.abs(pos - revpos2) < error) {
            rev.setPosition(revpos3);
        } else if (Math.abs(pos - revpos3) < error) {
            rev.setPosition(revpos1);
        } else {
            rev.setPosition(revpos1);
        }
    }


    public int functionshootstate=0;

    public void shoot() {
        if(!shooting) return;

        switch(functionshootstate){
            case 0:
                shootTimer.reset();
                push_tun.setPosition(.50);
                functionshootstate = 1;
                break;

            case 1:
                if(shootTimer.milliseconds() > 200){
                    push_tun.setPosition(.28);
                    functionshootstate = 2;
                }
                break;

            case 2:
                // gata, așteaptă reset extern
                break;
        }
    }

    public int nrBall = 0;
    public int nrBallBasic = 0;
    public boolean[] orderBall = {false , false , false};
    public boolean readytoshoot = false;
    public void storage_sort(){
        switch(nrBall) {
            case 0:
                if (distanceCM < distanta_proxsensorCM) {
                    if (getColor() == neededOrder[0]) {
                        revRight();
                        orderBall[0] = true;
                        nrBall++;
                    } else {
                        revLeft();
                        nrBall++;
                    }
                }
                break;
            case 1:
                if (distanceCM < distanta_proxsensorCM) {
                    if (getColor() == neededOrder[1]) {
                        orderBall[1] = true;
                    }
                    if (orderBall[0]) {
                        revRight();
                        nrBall++;
                    } else {
                        revLeft();
                        nrBall++;
                    }
                }
                break;
            case 2:
                if (distanceCM < distanta_proxsensorCM) {
                    if (getColor() == neededOrder[2]) {
                        orderBall[2] = true;
                    }
                    if (orderBall[1] == false && orderBall[0] == false) {
                        revLeft();
                    }
                }
                readytoshoot = true;
                break;
        }
        if (readytoshoot) {

            switch (shootState) {

                case 0:
                    while(functionshootstate<=1)
                        shoot();
                    functionshootstate=0;

                    shootTimer.reset();
                    shootState = 1;
                    break;

                case 1:
                    if (shootTimer.milliseconds() > 400) {
                        if (orderBall[1]) revRight();
                        else revLeft();
                        shootTimer.reset();
                        shootState = 2;
                    }
                    break;

                case 2:
                    if (shootTimer.milliseconds() > 300) {
                        while(functionshootstate<=1)
                            shoot();
                        functionshootstate=0;
                        shootTimer.reset();
                        shootState = 3;
                    }
                    break;

                case 3:
                    if (shootTimer.milliseconds() > 400) {
                        if (orderBall[1]) revRight();
                        else revLeft();
                        shootTimer.reset();
                        shootState = 4;
                    }
                    break;

                case 4:
                    if (shootTimer.milliseconds() > 300) {
                        while(functionshootstate<=1)
                            shoot();
                        functionshootstate=0;
                        shootState = 0;
                        readytoshoot = false;
                    }
                    break;
            }
        }

    }
    int basicState=0;
    int shoot_time = 500; //todo test value

    public void storage_basic(){
        if(distanceCM>distanta_proxsensorCM) return;
            revLeft();


    }
    public void orientare(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result=limelight.getLatestResult();
        if(result!=null&& result.isValid()){
            Pose3D botpose=result.getBotpose_MT2();
            double tx = result.getTx();

            double power = 0.2;

            if(tx > 1){ // dreapta
                stft.setPower(power);
                drft.setPower(-power);
                stspa.setPower(power);
                drspa.setPower(-power);
            } else if(tx < -1){ // stanga
                stft.setPower(-power);
                drft.setPower(power);
                stspa.setPower(-power);
                drspa.setPower(power);
            } else { // ținta centrată
                stft.setPower(0);
                drft.setPower(0);
                stspa.setPower(0);
                drspa.setPower(0);
            }


        }


    }












}
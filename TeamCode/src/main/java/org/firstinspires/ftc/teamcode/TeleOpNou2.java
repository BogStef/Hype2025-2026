

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



    public DcMotor intake_up;
    public DcMotor intake_down;

    public Servo push_tun;
    public Servo rev;

    public final double revpos1=0.22;
    public final double revpos2=0.58;
    public final double revpos3 = 0.96;



    public final double vmax = 0.7;
    public int motif_code;
    public float distanta_proxsensorCM=16;//real = 13.3

    public int[] neededOrder = {0,0,0};//0 - purple, 1 - green

    int shootState=0;

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

        tun1 = hardwareMap.get(DcMotor.class, "tun1");
        tun2 = hardwareMap.get(DcMotor.class, "tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setDirection(DcMotor.Direction.REVERSE);

        intake_up = hardwareMap.get(DcMotor.class, "intake_up");///intake
        intake_down = hardwareMap.get(DcMotor.class, "intake_down");
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");///gyro
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP))

        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");///camera
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);

        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");


        push_tun = hardwareMap.get(Servo.class,"push_tun");
        rev = hardwareMap.get(Servo.class,"rev");

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


    double p = 25;
    double i = 0;
    double d = 10;
    double f = 36;

    boolean sort;
    boolean shooting = false;

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

        if (starerun) {

            /*nghi = unghi() + resunghi;
            if (unghi < 0) {
                unghi = unghi + 360;
            }

            final double x = Math.pow(-gamepad1.left_stick_x, 3.0);
            final double y = Math.pow(gamepad1.left_stick_y, 3.0);

            final double rotation = Math.pow(gamepad1.right_stick_x * 0.8, 3);
            final double direction = Math.atan2(x, y) + Math.toRadians(unghi);
            final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

            stftpwr = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
            drftpwr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
            stspapwr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
            drspapwr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

            double raport=0.657;
            double vit=2787;



            if(gamepad2.right_trigger == 1)
            {
                stft.setVelocity(stftpwr*raport*vit - gamepad2.right_stick_y/4.5);
                drft.setVelocity(drftpwr*raport*vit - gamepad2.right_stick_y/4.5);
                stspa.setVelocity(stspapwr*raport*vit - gamepad2.right_stick_y/4.5);
                drspa.setVelocity(drspapwr*raport*vit - gamepad2.right_stick_y/4.5);
            }else
            {
                stft.setVelocity(stftpwr*raport*vit);
                drft.setVelocity(drftpwr*raport*vit);
                stspa.setVelocity(stspapwr*raport*vit);
                drspa.setVelocity(drspapwr*raport*vit);
            }*/
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

            double raport = 0.585; //experimental
            double vit = 2787; //435 rpm / 60 * 345.5 ticks/rev

            stft.setVelocity(stftpwr * raport * vit);
            drft.setVelocity(drftpwr * raport * vit);
            stspa.setVelocity(stspapwr * raport * vit);
            drspa.setVelocity(drspapwr * raport * vit);
            telemetry.update();
        } else {


            stft.setVelocityPIDFCoefficients(p, i, d, f);
            drft.setVelocityPIDFCoefficients(p, i, d, f);
            stspa.setVelocityPIDFCoefficients(p, i, d, f);
            drspa.setVelocityPIDFCoefficients(p, i, d, f);
            //PID

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

            double raport = 0.585; //experimental
            double vit = 2787; //435 rpm / 60 * 345.5 ticks/rev

            stft.setVelocity(stftpwr * raport * vit);
            drft.setVelocity(drftpwr * raport * vit);
            stspa.setVelocity(stspapwr * raport * vit);
            drspa.setVelocity(drspapwr * raport * vit);
            telemetry.update();


        }

        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);//update continuu la distanta


        if (motif_code == -1)  motif_code = motif(); //cauta motif
        else limelight.stop();

        ///OVVERIDEURI
        force_right();///PAD URI
        force_left();
        if(gamepad2.dpad_up){ /// DPAD_UP - SHOOT
            while(functionshootstate<=1){
                shoot();
            }
            functionshootstate=0;
        }
        //////////////////////////////////////////////////

        telemetry.addData("Detected Motif:",motif_code);
        telemetry.update();
        tun(.5);///0.367 ///square
        intake(.7); ///triunghi = on //cross = stop //circle= pass (parametru viteza de la 0 la 1)

        if(gamepad2.right_trigger>0.3){///SHOOT BASIC ///RIGHT TRIGGER
            while(basicState<=4)
                storage_basic();
            basicState=0;
        }




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
        }
        if (gamepad2.cross) {
            intake_up.setPower(0);
            intake_down.setPower(0);
        }

        if (gamepad2.circle) {
            intake_up.setPower(0);
            pass_intake = !pass_intake;
            if (pass_intake) {
                intake_down.setPower(vit);
            } else {
                intake_down.setPower(-1*vit);
            }
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

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
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
    public void revRight(){
        if(rev.getPosition()==revpos1)
            rev.setPosition(revpos3);
        else if(rev.getPosition()==revpos2)
            rev.setPosition(revpos1);
        else if(rev.getPosition()==revpos3)
            rev.setPosition(revpos2);
        else
            rev.setPosition(revpos1);
    }
    public void revLeft(){
        if(rev.getPosition()==revpos1)
            rev.setPosition(revpos2);
        else if(rev.getPosition()==revpos2)
            rev.setPosition(revpos3);
        else if(rev.getPosition()==revpos3)
            rev.setPosition(revpos1);
        else
            rev.setPosition(revpos1);
    }

    public int functionshootstate=0;

    public void shoot() {///face pac pac si face shooting true
        switch(functionshootstate){
            case 0:
                shootTimer.reset();
                push_tun.setPosition(.4);///
                functionshootstate++;
            case 1:
                if(shootTimer.milliseconds()>90){
                    push_tun.setPosition(0);
                    functionshootstate++;
                }

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
    int shoot_time = 20; //todo test value
    public void storage_basic(){
        if(distanceCM>=distanta_proxsensorCM) return;
        switch (basicState){
            case 0:
                while(functionshootstate<=1)
                    shoot();
                functionshootstate=0;

                shootTimer.reset();
                basicState++;
                break;
            case 1:
                if(shootTimer.milliseconds() > shoot_time) {
                    revLeft();
                    shootTimer.reset();
                    basicState++;
                }
                break;
            case 2:
                if(shootTimer.milliseconds() > shoot_time) {
                    while(functionshootstate<=1)
                        shoot();
                    functionshootstate=0;
                    shootTimer.reset();
                    basicState++;
                }
                break;
            case 3:
                if(shootTimer.milliseconds() > shoot_time) {
                    revLeft();
                    shootTimer.reset();
                    basicState++;
                }
                break;
            case 4:
                if(shootTimer.milliseconds() > shoot_time) {
                    while(functionshootstate<=1)
                        shoot();
                    functionshootstate=0;
                    shootTimer.reset();
                    basicState++;
                }
                break;
        }
        /*
        shoot
        revleft
        shoot
        revleft
        shoot
         */

    }
    public void orientare(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult result=limelight.getLatestResult();
        if(result!=null&& result.isValid()){
            Pose3D botpose=result.getBotpose_MT2();
            double tx = result.getTx();



        }

    }












}
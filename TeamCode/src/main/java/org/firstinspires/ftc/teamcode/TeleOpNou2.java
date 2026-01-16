

package org.firstinspires.ftc.teamcode;

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
    public boolean toggle_tun;
    public DistanceSensor rangeSensor;



    public DcMotor intake_up;
    public DcMotor intake_down;

    public Servo push_tun;
    public Servo rev;




    public final double vmax = 0.7;
    public int motif_code;

    public int[] neededOrder = {};//0 - purple, 1 - green

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
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");

        push_tun = hardwareMap.get(Servo.class,"push_tun");
        rev = hardwareMap.get(Servo.class,"rev");

        pass_intake = false;///pass intake
        toggle_tun = false;
        motif_code = -1;
        sort=false;//daca intram in modul de sortare(la finalul meciului)




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
    public void updateShoot() {
        if (shooting && shootTimer.milliseconds() > 300) {
            push_tun.setPosition(0.0);
            shooting = false;
        }
    }
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
        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
        if (motif_code == -1)
            motif_code = motif();
        tun(.7);
        intake(.7);
        ///TODO if(gamepad2.right_trigger>0) sort=true;
        if(sort){
            storage_sort();
        }
        else{
            storage_basic();
        }
        updateShoot();
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

    void intake(double vit) {
        {
        }
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
                intake_down.setPower(-vit);
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
                switch(id){
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
                }
                return id;


            }
        }

        return -1;
    }

    int caterun = 0;

    public double unghi() {
        imu = hardwareMap.get(IMU.class, "imu");
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
        rev.setPosition(rev.getPosition()-Math.toRadians(120)/Math.PI);
    }
    public void revLeft(){
        rev.setPosition(rev.getPosition()+Math.toRadians(120)/Math.PI);
    }
    public void shoot() {
        push_tun.setPosition(1.0);
        shootTimer.reset();
        shooting = true;
    }
    public int nrBall = 0;
    public int nrBallBasic = 0;
    public boolean[] orderBall = {false , false , false};
    public void storage_sort(){
        switch(nrBall) {
            case 0:
                if (distanceCM < 0) {//TODO valori
                    if (getColor() == neededOrder[0]){
                        revRight();
                        orderBall[0] = true;
                        nrBall++;
                    }
                    else{
                        revLeft();
                        nrBall++;
                    }
                }
                break;
            case 1:
                if (distanceCM < 0) {//TODO valori
                    if (getColor() == neededOrder[1]) {
                        orderBall[1] = true;
                    }
                    if(orderBall[0]){
                        revRight();
                        nrBall++;
                    }
                    else{
                        revLeft();
                        nrBall++;
                    }
                }
                break;
            case 2:
                if(distanceCM < 0){//TODO valori
                    if(getColor() == neededOrder[2]){
                        orderBall[2] = true;
                    }
                    if(orderBall[1] == false && orderBall[0] == false){
                        revLeft();
                    }
                }
                break;
        }
        if(orderBall[1]){
            shoot();
            revRight();
            shoot();
            revRight();
            shoot();
        }
        else{
            shoot();
            revLeft();
            shoot();
            revLeft();
            shoot();
        }
    }


    public void storage_basic(){
        switch(nrBallBasic) {
            case 0:
                if (distanceCM < 0) {//TODO
                    revLeft();
                    //shoot
                    nrBallBasic++;
                }
            case 1:
                if (distanceCM < 0) {//TODO
                    revLeft();
                    //shoot
                    nrBallBasic++;
                }
            case 2:
                if (distanceCM < 0) {//TODO
                    revLeft();
                    //shoot
                    nrBallBasic++;
                }
        }
    }












}
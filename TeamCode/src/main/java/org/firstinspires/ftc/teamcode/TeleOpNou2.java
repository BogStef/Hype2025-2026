

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    public DcMotorEx tun1;
    public DcMotorEx tun2;
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



    //public DcMotor intake_up;
    public DcMotor intake;

    public Servo push_tun;
    public Servo rev;

    //public CRServo improvizatie_intake;

    public final double revpos1=0.00;
    public final double revpos2=0.37;
    public final double revpos3 = 0.76;



    public final double vmax = 0.7;
    public int motif_code;
    public double distanta_proxsensorCM=6.7;//real = 13.3

    public int[] neededOrder = {0,0,0};//0 - purple, 1 - green
    public DcMotor encoder;
    public int encoder_ticks;
    public double encoder_truepos;
    public int encoder_ticks_needed;
    public boolean revBusy = false;
    public double  encoder_currentPos;
    int shootState=0;
    int color;
    double red, blue, green, alpha, error;
    double vitRev;
    double targetAngle;
    double speedDown;
    double ticksPerStep = 8192 / 3;
    int index, target;
    boolean pressedUp;
    int motif_switch = 0;
    public int getColor() { /// 0==mov 1==green
        //double red, blue, green, alpha, error;
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
//        stspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        stft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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



        tun1 = hardwareMap.get(DcMotorEx.class, "tun1");
        tun2 = hardwareMap.get(DcMotorEx.class, "tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun1.setDirection(DcMotor.Direction.REVERSE);
        tun1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tun2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake_up = hardwareMap.get(DcMotor.class, "intake_up");///intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake_up.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu_expansion");///gyro
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN))

        );

        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");///camera
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);


        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");


        push_tun = hardwareMap.get(Servo.class,"push_tun");
        push_tun.setDirection(Servo.Direction.REVERSE);
        rev = hardwareMap.servo.get("rev");

        pass_intake = false;///pass intake
        toggle_tun = false;
        motif_code = -1;
        sort=false;//daca intram in modul de sortare(la finalul meciului)
        revBusy = false;
        vitRev = 0.5;
        index = 0;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));///todo

        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");

        encoder = hardwareMap.dcMotor.get("ThroughBoreEncoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tun_test=0;
    }
    boolean pressedl = false;
    int tun_test;
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

    boolean lastDown;
    boolean lastDown1;
    boolean lastDown2;
    double trueTa = 0.00;

    //PID
    public void calculateDistTun() {
        double ta = 0.00;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            ta = result.getTa();
        }
        else ta=0;
        trueTa=ta; // ne folosim numai de ta pt ca la zalau vom trage numa de pe linie ca urmare unghiul nu ne fut
    }

    /*
    * nu merge pt ca DT inca e in hypemetrii si h e in cm la fel si q
    * public double calcVitezaTun () {
            double vitTun=0;
            double cosB, sinB, Q=1, marginOfError = 1;
            double DT = calculateDistTun();
            cosB = Math.sqrt(1-(75/DT)*(75/DT));
            sinB = 1 - (cosB*cosB);
            vitTun = Math.sqrt(9.81 * (DT*cosB+Q) /sinB /sinB /sinB * marginOfError);

            return vitTun;
        }
    * */



    double speed,turn,strafe;
    double vitTun = 1;
    double vitTun2 = 1;
    public void run() {///TODO clean this up
        telemetry.update();



        boolean isTriggerPressedb = gamepad1.circle;
        if (isTriggerPressedb && !pressedb) { //circle
            starerun = !starerun;
        }
        pressedb = isTriggerPressedb;

        double stftpwr;
        double stspapwr;
        double drftpwr;
        double drspapwr;
        speed = gamepad1.left_stick_y;

        turn = gamepad1.right_stick_x;
        if(disable_driver_turn) turn =0;
        strafe = gamepad1.left_stick_x;

        telemetry.addData("raw unghi",imu.getRobotYawPitchRollAngles());
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if(starerun){
            double tempStrafe = speed * Math.cos(-heading) - strafe * Math.sin(-heading);
            double tempSpeed = speed * Math.sin(-heading) + strafe * Math.cos(-heading);
            speed=tempSpeed;
            strafe = tempStrafe;
            telemetry.addData("gyro",starerun);
        }


        stftpwr = speed + turn + strafe;
        drftpwr = speed - turn - strafe;
        stspapwr = speed + turn - strafe;
        drspapwr = speed - turn + strafe;

//        double raport =0.585; /// 0.585; //experimental
//        double vit = 2787; //435 rpm / 60 * 345.5 ticks/rev
//        double tolerance = Math.toRadians(1.0);
        /*
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
        }*/

        double maxspeed=0.5;
        stft.setPower(stftpwr*maxspeed);
        drft.setPower(drftpwr*maxspeed);
        stspa.setPower(stspapwr*maxspeed);
        drspa.setPower(drspapwr*maxspeed);

        ////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////


        if (motif_code == -1)  motif_code = motif(); //cauta motif




        /// OVVERIDE LA SHOOT
        boolean currentrighttrigger=gamepad2.right_bumper;
        if(currentrighttrigger && !lastrighttrigger){
            shooting=true;
        }
        lastrighttrigger=currentrighttrigger;
        /////////

        if(shooting){
            shoot();
            //tun(vitTun);
            if(functionshootstate == 2 && !storage_basic_running && !readytoshoot){
                shooting = false;
                functionshootstate = 0;
            }
        }
        /////////////////////
        //////////////////
        ///////////////////
        /////////////////////
        ///////////////////

        ////////



        intake(0.5); ///triunghi = on //cross = stop
        //rev.setPosition((1-gamepad2.right_stick_y)/2);

        if (gamepad2.right_trigger > 0.3 && !storage_basic_running) { /// pornim storage basic
            storage_basic_running = true;
            basic_state = 0;
            timer_storage_basic.reset();
            storage_basic();
        }
        if(gamepad2.left_trigger>0.3){//stop fortat la basic storage
            storage_basic_running=false;
            basic_state=0;
            timer_storage_basic.reset();
        }
        if(storage_basic_running) storage_basic();



        /// ORIENTARE
        // apasa driver1 oe a
        boolean currentA = gamepad1.square;

        if (currentA && !aPrev) {
            alignRunning = !alignRunning;
            filteredTx = 0;
            lastAlignPower = 0;
            lastSeenDirection = 0;
            orientareGata = false;
        }

        aPrev = currentA;

        if (alignRunning) {
            orientare();
            telemetry.addLine("ALIIGNING ON");
        }
        else{
            disable_driver_turn=false;
            telemetry.addLine("ALLIGNING OFF");
        }
        ////////////////


        calculateDistTun();
        double const1 = 310, const2 = 900;
        if(trueTa!=0){
            vitTun = const1/trueTa + const2;
            vitTun2 = const1/trueTa +const2+300;
        }
        else {
            vitTun = const2;
            vitTun2 = const2+670;
        }

        telemetry.addData("vittun",vitTun);


        tun(vitTun,vitTun2);
        tun1.setVelocityPIDFCoefficients(6.7,0,0.01,11.7);
        tun2.setVelocityPIDFCoefficients(6.7,0,0.01,11.7);

        boolean currentDown1 = gamepad2.dpad_right;
        if(currentDown1 && !lastDpadUp){
            readytoshoot = !readytoshoot;
           // telemetry.addLine("revRight");
        }
        lastDpadUp = currentDown1;

        boolean currentUp = gamepad2.dpad_up;
        if (currentUp && !lastDown) {
            sort = !sort;
            nrBall=0;
            sort_timer.reset();
        }
        lastDown = currentUp;
        if (sort) storage_sort();

        boolean currentDown = gamepad2.dpad_down;
        if (currentDown && !lastDown1) {
            switch (motif_switch) {
                case 0: neededOrder[0] = 1;
                    neededOrder[1] = 0;
                    neededOrder[2] = 0;
                    motif_switch++;
                    break;
                case 1: neededOrder[0] = 0;
                    neededOrder[1] = 1;
                    neededOrder[2] = 0;
                    motif_switch++;
                    break;
                case 2: neededOrder[0] = 0;
                    neededOrder[1] = 0;
                    neededOrder[2] = 1;
                    motif_switch=0;
                    break;
            }
        }
        lastDown1 = currentDown;
        boolean currentDown2 = gamepad2.dpad_left;
        if(currentDown2 && !lastDown2){
            revLeft();
           // telemetry.addLine("revleft");
        }
        lastDown2 = currentDown2;
        color = getColor();
        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
        //telemetry.addData("revpos",rev.getPosition());
        telemetry.addData("motif",motif_code);
        //telemetry.addData("basicstate",basic_state);
        //telemetry.addData("shooting ",shooting);
        //telemetry.addData("shoot state", functionshootstate);
        telemetry.addData("color ", color);
        //telemetry.addData("red",colorsensor.red());
        //telemetry.addData("green",colorsensor.green());
        //telemetry.addData("blue", colorsensor.blue());
        //telemetry.addData("alpha", colorsensor.alpha());
        //telemetry.addData("distanta ",distanceCM);
        //telemetry.addData("ta",trueTa);
        //PIDFCoefficients defaultpidftun=tun1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.addData("pidf", defaultpidftun);
        telemetry.addData("sorting",sort);
        telemetry.addData("sorting state",nrBall);
        telemetry.addData("motif switch",motif_switch);
        telemetry.addData("ball 1",orderBall[0]);
        telemetry.addData("ball 2",orderBall[1]);
        telemetry.addData("ball 3",orderBall[2]);
        telemetry.addData("ready to shoot",readytoshoot);
        telemetry.addData("shoot state",shootState);
        telemetry.update();
    }

    boolean lastSquare = false, lastLeftArrow = false, lastRightArrow = false; // llA si lrA sunt temp pt testat

    void tun(double v1,double v2) {
        boolean currentSquare = gamepad2.square;
        if (currentSquare && !lastSquare) {
            toggle_tun = !toggle_tun;
        }
        lastSquare = currentSquare;
        if (toggle_tun) {
            tun1.setVelocity(v1);
            tun2.setVelocity(v2);
        } else {
            tun1.setVelocity(0);
            tun2.setVelocity(0);
        }

        telemetry.addData("vit:",vitTun);
    }






    ///triunghi = on //cross = stop //circle= pass (parametru viteza de la 0 la 1)
    ///
    ///
    ///
    boolean ballDetected = false;
    boolean lastBallDetected = false;
    int ballCount = 0;
    boolean toggle_intake = false;
    boolean currenttriangle = false;
    boolean lasttriangle = false;
    ElapsedTime IntakeTimer = new ElapsedTime();

    void intake(double vit) {

        currenttriangle = gamepad2.triangle;
        if (currenttriangle && !lasttriangle) {
            toggle_intake = !toggle_intake;

            if (toggle_intake) {
                ballCount = 0;   // reset la pornire intake
            }
        }
        lasttriangle = currenttriangle;

        // pornire motor intake
        if (toggle_intake) {
            intake.setPower(vit);
        } else {
            intake.setPower(0);
        }

        // detectare bila
        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
        ballDetected = distanceCM < distanta_proxsensorCM;

        if (ballDetected && !lastBallDetected && ballCount < 3 && IntakeTimer.milliseconds()>300) {

            revLeft();   // muta revolver o poziție
            ballCount++;
            IntakeTimer.reset();
        }

        lastBallDetected = ballDetected;
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
                motif_code = id;
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


    public void rotation() {
        final double rot = gamepad1.right_stick_x;

        drft.setVelocity(2897 * -vmax * rot);
        stft.setVelocity(2897 * vmax * rot);
        drspa.setVelocity(2897 * -vmax * rot);
        stspa.setVelocity(2897 * vmax * rot);
    }

    public void get_encoder_truepos(){
        encoder_ticks = encoder.getCurrentPosition();
        int mod = encoder_ticks % 8192;
        if (mod<0) mod += 8192;
        encoder_truepos = mod;
    }

        /*public void stop_revolver(){
            if (!revBusy) return;
            get_encoder_truepos();
            speedDown = speedDown * 0.99 ;
            rev.setPower(speedDown);
            if (rev.getPower()>0) {
                if (encoder_truepos >= encoder_currentPos) {
                    rev.setPower(0);
                    revBusy = false;
                }
            }
            else if (rev.getPower()<0) {
                if (encoder_truepos <= encoder_currentPos) {
                    rev.setPower(0);
                    revBusy = false;
                }
            }
        }*/
    ///TODO make continuous rotation && find pos with through bore encoder
        /*public void revRight(){
            if (!revBusy) {
                get_encoder_truepos();
                encoder_currentPos = encoder_truepos;
                speedDown = vitRev;
                rev.setPower(vitRev);
                revBusy = true;
            }
        }

        public void revLeft(){
            if (!revBusy) {
                get_encoder_truepos();
                encoder_currentPos = encoder_truepos;
                speedDown = -vitRev;
                rev.setPower(-vitRev);
                revBusy = true;
            }
        }*/

    public void revLeft() {
        double err = 0.01;
        double current = rev.getPosition();

        if (Math.abs(current - revpos1) < err) {
            rev.setPosition(revpos2);
        }
        else if (Math.abs(current - revpos2) < err) {
            rev.setPosition(revpos3);
        }
        else if (Math.abs(current - revpos3) < err) {
            rev.setPosition(revpos1);
        }
        else {
            // Default if position doesn't match any
            rev.setPosition(revpos1);
        }
    }

    public void revRight(){
        double err = 0.01;
        double current = rev.getPosition();

        if (Math.abs(current - revpos1) < err) {
            rev.setPosition(revpos3);
        }
        else if (Math.abs(current - revpos2) < err) {
            rev.setPosition(revpos1);
        }
        else if (Math.abs(current - revpos3) < err) {
            rev.setPosition(revpos2);
        }
        else {
            // Default if position doesn't match any
            rev.setPosition(revpos1);
        }
    }
        /*public void old_revRight() {
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

        public void old_revLeft() {
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
    */

    public int functionshootstate=0;

    public void shoot() {
        if(!shooting) return;

        switch(functionshootstate){
            case 0: // start shoot
                push_tun.setPosition(0); // retract
                shootTimer.reset();       // reset timer once
                functionshootstate = 1;
                break;

            case 1: // extend after 200ms
                if(shootTimer.milliseconds() > 200){
                    push_tun.setPosition(0.75); // extend
                    functionshootstate = 2;
                }
                break;

            case 2: // done, wait for external reset
                break;
        }
    }


    public int nrBall = 0;
    public int nrBallBasic = 0;
    ElapsedTime sort_timer = new ElapsedTime();
    int sortDelay = 350;
    public boolean[] orderBall = {false , false , false};
    public boolean readytoshoot = false;
    ///todo add timer or fix it
    public void storage_sort(){
        switch(nrBall) {
            case 0:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    if (getColor() == neededOrder[0]) {
                        revLeft();
                        orderBall[0] = true;
                        nrBall++;
                    } else {
                        revRight();
                        orderBall[0] = false;
                        nrBall++;
                    }
                    sort_timer.reset();
                }
                break;
            case 1: if (sort_timer.milliseconds() > sortDelay) {
                nrBall++;
            }
            break;
            case 2:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    if (getColor() == neededOrder[1]) {
                        orderBall[1] = true;
                    }
                    else orderBall[1] = false;
                    if (orderBall[0]) {
                        revLeft();
                        nrBall++;
                    } else {
                        revRight();
                        nrBall++;
                    }
                    sort_timer.reset();
                }
                break;
            case 3: if (sort_timer.milliseconds() > sortDelay) {
                nrBall++;
            }
                break;
            case 4:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    if (getColor() == neededOrder[2]) {
                        orderBall[2] = true;
                    }
                    else orderBall[2] = false;
                    if (orderBall[1] == false && orderBall[0] == false) {
                        revRight();
                    }
                    nrBall++;
                    sort_timer.reset();
                    shootState=0;
                }
                break;
        }
        if (readytoshoot) {

            switch (shootState) {

                case 0:
                    if (sort_timer.milliseconds() > sortDelay) {
                        if (!storageShooting) {
                            storageShooting = true;
                            shooting = true;
                            shoot();
                        }
                        if (functionshootstate == 2) {
                            shooting = false;
                            storageShooting = false;
                            functionshootstate = 0;
                            shootState++;
                            sort_timer.reset();
                        }
                    }
                    break;

                case 1:
                    if (sort_timer.milliseconds() > sortDelay) {
                        if (orderBall[1]) revRight();
                        else revLeft();
                        sort_timer.reset();
                        shootState = 2;
                    }
                    break;

                case 2:
                    if (sort_timer.milliseconds() > sortDelay) {
                        if (!storageShooting) {
                            storageShooting = true;
                            shooting = true;
                            shoot();
                        }
                        if (functionshootstate == 2) {
                            shooting = false;
                            storageShooting = false;
                            functionshootstate = 0;
                            shootState++;
                            sort_timer.reset();
                        }
                    }
                    break;

                case 3:
                    if (sort_timer.milliseconds() > sortDelay) {
                        if (orderBall[1]) revRight();
                        else revLeft();
                        sort_timer.reset();
                        shootState = 4;
                    }
                    break;

                case 4:
                    if (sort_timer.milliseconds() > sortDelay) {
                        if (!storageShooting) {
                            storageShooting = true;
                            shooting = true;
                            shoot();
                        }
                        if (functionshootstate == 2) {
                            shooting = false;
                            storageShooting = false;
                            functionshootstate = 0;
                            shootState++;
                            sort_timer.reset();
                        }
                        nrBall=0;
                    }
                    break;
            }
        }

    }


    int basic_state=0;
    boolean storage_basic_running = false;
    ElapsedTime timer_storage_basic=new ElapsedTime();

    int revDelay = 350;   // pauza după revLeft (ms)
    int shootDelay = 400; // cât ținem push_tun apăsat (ms)

    boolean storageShooting = false; // new flag to indicate storage-controlled shooting


    public void storage_basic() {
        if (!storage_basic_running) return;

        switch (basic_state) {

            // 1. First revLeft
            case 0:
                revLeft();
                timer_storage_basic.reset();
                basic_state++;
                break;

            // 2. Pause after first revLeft
            case 1:
                if (timer_storage_basic.milliseconds() > revDelay) {
                    basic_state++;
                }
                break;

            // 3. First shoot
            case 2:
                if (!storageShooting) {
                    storageShooting = true;
                    shooting = true;
                    shoot(); // start shooting
                }
                if (functionshootstate == 2) { // shoot finished
                    shooting = false;
                    storageShooting = false;
                    functionshootstate = 0;
                    timer_storage_basic.reset();
                    basic_state++;
                }
                break;

            // 4. Pause after first shoot
            case 3:
                if (timer_storage_basic.milliseconds() > shootDelay) {
                    basic_state++;
                }
                break;

            // 5. Second revLeft
            case 4:
                revLeft();
                timer_storage_basic.reset();
                basic_state++;
                break;

            // 6. Pause after second revLeft
            case 5:
                if (timer_storage_basic.milliseconds() > revDelay) {
                    basic_state++;
                }
                break;

            // 7. Second shoot
            case 6:
                if (!storageShooting) {
                    storageShooting = true;
                    shooting = true;
                    shoot();
                }
                if (functionshootstate == 2) {
                    shooting = false;
                    storageShooting = false;
                    functionshootstate = 0;
                    timer_storage_basic.reset();
                    basic_state++;
                }
                break;

            // 8. Pause after second shoot
            case 7:
                if (timer_storage_basic.milliseconds() > shootDelay) {
                    basic_state++;
                }
                break;

            // 9. Third revLeft
            case 8:
                revLeft();
                timer_storage_basic.reset();
                basic_state++;
                break;

            // 10. Pause after third revLeft
            case 9:
                if (timer_storage_basic.milliseconds() > revDelay) {
                    basic_state++;
                }
                break;

            // 11. Third (final) shoot
            case 10:
                if (!storageShooting) {
                    storageShooting = true;
                    shooting = true;
                    shoot();
                }
                if (functionshootstate == 2) {
                    shooting = false;
                    storageShooting = false;
                    functionshootstate = 0;
                    basic_state = 11; // finished
                    storage_basic_running = false;
                }
                break;

            case 11:
                // storage finished
                storage_basic_running = false;
                break;
        }
    }




    boolean alignRunning = false;
    boolean aPrev = false;
    double filteredTx = 0;
    double lastAlignPower = 0;
    boolean orientareGata = false;
    double lastSeenDirection=1;
    double searchPower = 0.003;
    boolean search =false;
    boolean disable_driver_turn=false;
    ElapsedTime searchTimer = new ElapsedTime();

    public void orientare() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            disable_driver_turn=false;
            return;
        }
        search = false;
        disable_driver_turn=true;

        double rawTx = result.getTx();
        filteredTx = 0.9 * filteredTx + 0.1 * rawTx;
        if(Math.abs(filteredTx)>1)
            lastSeenDirection = filteredTx;
        // ===== PARAMETRI FINI =====

        double kP = 0.0022;
        double maxPower = 0.1;
        double deadband = 6 ;
        double ALIGN_SPEED = 0.8 ;

        double absTx = Math.abs(filteredTx);

        if (absTx < deadband) {
            stopDrive();
            lastAlignPower = 0;
            orientareGata = true;
            return;
        }

        double scale;//setam viteza de rotatie in functie de unghi
        //daca unghiu e mare(adica e rau) atunci ne miscam repede
        //daca unghiu e mic (adica aproape perfect) atunci ne miscam inet
        //        if (absTx > 6) scale = 1.0;
        //        else if (absTx > 2) scale = 0.5;
        //        else scale = 0.25;
        scale = 1/27 * (absTx*absTx) + 0.15; /// scale(absTx) = 1/30 * absTx^2+0.05 (functie)
        if(scale>1) scale=1;

        double desiredPower = kP * filteredTx * scale;
        desiredPower = Range.clip(desiredPower, -maxPower, maxPower);

        // LIMITARE ACCELERAȚIE
        double maxDelta = 0.0015;
        double power = Range.clip(
                desiredPower,
                lastAlignPower - maxDelta,
                lastAlignPower + maxDelta
        );

        power *= ALIGN_SPEED;
        lastAlignPower = power;
        if(Math.abs(speed)<0.05&&Math.abs(strafe)<0.05&&Math.abs(turn)<0.5){
            stft.setPower(power);
            drft.setPower(-power);
            stspa.setPower(power);
            drspa.setPower(-power);
        }


    }




    private void stopDrive() {
        stft.setPower(0);
        drft.setPower(0);
        stspa.setPower(0);
        drspa.setPower(0);
    }




}
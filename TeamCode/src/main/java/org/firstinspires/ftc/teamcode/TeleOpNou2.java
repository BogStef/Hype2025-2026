

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
    public final double revpos1=0.06;
    public final double revpos2=0.45;
    public final double revpos3 = 0.82;
    public final double revposAlter = 0.056;
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
    int shootState=-1;
    int color;
    double red, blue, green, alpha;
    double vitRev;
    double targetAngle;
    double speedDown;
    double ticksPerStep = 8192 / 3;
    int index, target;
    boolean pressedUp;
    int motif_switch = 23;
    boolean lastLeftTrigger;
    boolean lastBumper;

    public int getColor() { /// 0==mov 1==green
        //double red, blue, green, alpha, error;
        double error;
        error = 120;
        red = colorsensor.red();
        blue = colorsensor.blue();
        green = colorsensor.green();
        alpha = colorsensor.alpha();
        if(green>95&&green>blue&&green>red){
            return 1;}
        if(red>green||blue>green)
            return 0;
        return -1;

    }
    @Override
    public void runOpMode() {
        initialize();
        telemetry.update();
        waitForStart();
        imu.resetYaw();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                run();
            }
        }
    }


    public void initialize() {
        stft = hardwareMap.get(DcMotorEx.class, "stft");///miscare
        drft = hardwareMap.get(DcMotorEx.class, "drft");
        stspa = hardwareMap.get(DcMotorEx.class, "stspa");
        drspa = hardwareMap.get(DcMotorEx.class, "drspa");

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
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");

        encoder = hardwareMap.dcMotor.get("ThroughBoreEncoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        tun1.setVelocityPIDFCoefficients(6.7,0,0.1,13.7);
        tun2.setVelocityPIDFCoefficients(6.7,0,0.1,13.7);
    }
    boolean starerun = true;
    boolean pressedb = false;
    double p = 10;//25
    double i = 0;
    double d = 5;//10
    double f = 1670;//36

    boolean sort;
    boolean shooting = false;
    boolean lastrighttrigger=false;
    boolean lastDown1;
    boolean lastDown2;
    double trueTa = 0.00;
    double trueTx=0.00;


    //PID
    public void calculateTrueTa() {
        double ta = 0.00;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            ta = result.getTa();
        }
        else ta=0;
        trueTa=ta; // ne folosim numai de ta pt ca la zalau vom trage numa de pe linie ca urmare unghiul nu ne fut
    }
    public void calculateTrueTx(){
        double Tx = 0.00;
        LLResult result = limelight.getLatestResult();;
        if(result!=null && result.isValid()){
            Tx = result.getTx();
        }
        else Tx=0.0;
        trueTx = Tx;
    }

    double vitTun = 1;
    double vitTun2 = 1;

    public void drive(double speed,double turn,double strafe,double maxspeed){
        double stftpwr,drftpwr,stspapwr,drspapwr;
        stftpwr = speed + turn + strafe;
        drftpwr = speed - turn - strafe;
        stspapwr = speed + turn - strafe;
        drspapwr = speed - turn + strafe;
        stft.setPower(stftpwr*maxspeed);
        drft.setPower(drftpwr*maxspeed);
        stspa.setPower(stspapwr*maxspeed);
        drspa.setPower(drspapwr*maxspeed);
    }

    double kP = 0.03;
    double error = 0;
    double lastError = 0;
    double angleTolerance = 1.5;
    double kD = 0.006;
    double curTime = 0;
    double lastTime = 0;
//    double [] stepSizes ={0.1,0.001,0.0001};
//    int stepIndex = 2;
    public void allign(double goalX){
        double tx = 0;
        if(gamepad1.left_trigger>.2){
            LLResult result = limelight.getLatestResult();
            List<FiducialResult> tags = result.getFiducialResults();
            if (result == null || !result.isValid()|| tags.isEmpty()) {
                lastTime = getRuntime();
                lastError = 0;
                drive(speed,turn,strafe,maxspeed);
                return;
            }

            for (FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (id == 24 || id == 20 ) {
                    error = goalX - result.getTx();
                    telemetry.addLine("ALLINING");
                    if(Math.abs(error) < angleTolerance){
                        drive(speed,0,strafe,maxspeed);
                    }
                    else{
                        double pTerm = error *kP;
                        curTime = getRuntime();
                        double dT = curTime - lastTime;
                        double dTerm = ((error-lastError)/dT)*kD;
                        drive(speed,Range.clip((pTerm+dTerm)*-1, - 0.4, 0.4),strafe,maxspeed);
                        lastError = error;
                        lastTime = curTime;
                    }
                }
                else{
                    lastTime = getRuntime();
                    lastError = 0;
                    drive(speed,turn,strafe,maxspeed);
                    return;
                }
            }
        }
        else{
            lastError = 0;
            lastTime = getRuntime();
            drive(speed,turn,strafe,maxspeed);
        }
    }

    final double maxspeed = 1;
    boolean pushTunInit = false;
    double speed,turn,strafe;
    public void run() {
        if (!pushTunInit) {
            push_tun.setPosition(0.73);
            pushTunInit=true;
        }
        boolean isTriggerPressedb = gamepad1.circle;
        if (isTriggerPressedb && !pressedb) { //circle
            starerun = !starerun;
            if(starerun){
                gamepad1.rumble(600);
            }
        }
        pressedb = isTriggerPressedb;


        speed = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        telemetry.addData("raw unghi",imu.getRobotYawPitchRollAngles());
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if(starerun){ ///toggle field centric drive

            double tempStrafe = strafe * Math.cos(heading) + speed * Math.sin(heading);
            double tempSpeed = -strafe * Math.sin(heading) + speed * Math.cos(heading);

            speed  = tempSpeed;
            strafe = tempStrafe;
            telemetry.addData("gyro", starerun);
        }
        if(gamepad1.right_trigger>0.2){
            drive(speed,turn,strafe,maxspeed+0.3);
        }
        else if(gamepad1.left_trigger<0.2)
           drive(speed,turn,strafe,maxspeed);

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
        /////////////////////
        /////////////////////
        /////////////////////
        /////////////////////
        ////////////////////

        if(gamepad1.triangle){
            imu.resetYaw();
        }

        intake(.7); /// triangle

        if (gamepad2.right_trigger > 0.3 && !storage_basic_running) { /// pornim storage basic
            storage_basic_running = true;
            basic_state = 0;
            timer_storage_basic.reset();
            storage_basic();
        }
        if(storage_basic_running) storage_basic();

        //starts the sorting process, press again to turn off
        boolean ltPressed;
        if(gamepad2.left_trigger>0.3){ //stop fortat la basic storage
            ltPressed = true;
        }
        else ltPressed = false;

        if (ltPressed && !lastLeftTrigger) {
            sort = !sort;
            sortState=0;
            sort_timer.reset();
        }
        lastLeftTrigger = ltPressed;

        //shoots the sorted balls once they are sorted

        boolean curBumper = gamepad2.left_bumper;
        if (curBumper && !lastBumper) {
            readytoshoot= !readytoshoot;
        }
        lastBumper = curBumper;

        if (sort) storage_sort();



        /// ORIENTARE
        // driver 1 apasa pe left trigger
        allign(-1.5);
        ////////////////
        /// ///////////


        calculateTrueTa();
        calculateTrueTx();
        double const1 =  670, const2 = 900;//
        if(trueTa!=0){
            vitTun = const1/trueTa + const2;
            if(trueTa<0.5) /// daca e departe
                vitTun2 = const1/trueTa +const2+1800;
            else /// daca e aproape
                vitTun2 = const1/trueTa +const2 + 700;

        }
        else {
            vitTun = const2;
            vitTun2 = const2+1000;
        }

        telemetry.addData("vittun",vitTun);
        telemetry.addData("vittunfinal",vittun1_final);
        tun(1000,4100);



//        boolean currentDown = gamepad2.dpad_down;
//        if (currentDown && !lastDown1) {
//            switch (motif_switch) {
//                case 23: neededOrder[0] = 1;
//                    neededOrder[1] = 0;
//                    neededOrder[2] = 0;
//                    motif_switch=21;
//                    break;
//                case 21: neededOrder[0] = 0;
//                    neededOrder[1] = 1;
//                    neededOrder[2] = 0;
//                    motif_switch=22;
//                    break;
//                case 22: neededOrder[0] = 0;
//                    neededOrder[1] = 0;
//                    neededOrder[2] = 1;
//                    motif_switch=23;
//                    break;
//            }
//        }
//        lastDown1 = currentDown;

        boolean currentDown1 = gamepad2.dpad_right;
        if(currentDown1 && !lastDpadUp){
            revRight();
        }
        lastDpadUp = currentDown1;

        boolean currentDown2 = gamepad2.dpad_left;
        if(currentDown2 && !lastDown2){
            revLeft();
            // telemetry.addLine("revleft");
        }
        lastDown2 = currentDown2;
        color = getColor();
        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("motif",motif_code);
        telemetry.addData("sorting",sort);
        telemetry.addData("sorting state",sortState);
//        telemetry.addData("motif switch",motif_switch);
//        telemetry.addData("ball 1",orderBall[0]);
//        telemetry.addData("ball 2",orderBall[1]);
//        telemetry.addData("ball 3",orderBall[2]);
        telemetry.addData("ready to shoot",readytoshoot);
        telemetry.addData("shoot state",shootState);
        telemetry.addData("TA",trueTa);
        telemetry.addData("tx",trueTx);
        telemetry.addData("revpos",rev.getPosition());
        telemetry.update();

    }

    boolean lastSquare = false, lastLeftArrow = false, lastRightArrow = false; // llA si lrA sunt temp pt testat
    double vittun1_final = 0;
    double vittun2_final = 0;
    void tun(double v1,double v2) {
        boolean currentSquare = gamepad2.square;

        if (currentSquare && !lastSquare) {
            toggle_tun = !toggle_tun;
            vittun1_final = v1;
            vittun2_final = v2;

        }
        lastSquare = currentSquare;

        if (gamepad2.circleWasPressed()) {
            vittun1_final = v1;
            vittun2_final = v2;
        }

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
    boolean toggle_intake_reverse;
    ElapsedTime IntakeTimer = new ElapsedTime();
    ElapsedTime IntakeReverseTimer = new ElapsedTime();
    int intakeReverseDelay = 3000;
    boolean rumbleDone = true;

    void intake(double vit) {

        currenttriangle = gamepad2.triangle;
        if (currenttriangle && !lasttriangle) {
            toggle_intake = !toggle_intake;

            if (toggle_intake) {
                ballCount = 0;   // reset la pornire intake
            }
        }
        lasttriangle = currenttriangle;

        if(gamepad2.crossWasPressed()){
            toggle_intake_reverse = !toggle_intake_reverse;
            IntakeReverseTimer.reset();
        } else if (IntakeReverseTimer.milliseconds() > intakeReverseDelay) { //asteapta o secunda si dupaia trece inapoi la normal
            toggle_intake_reverse = false;
        }


        // pornire motor intake
        if (toggle_intake) {
            if (!toggle_intake_reverse)
            intake.setPower(vit);
            else intake.setPower(-vit);

        } else {
            intake.setPower(0);
        }
        // detectare bila
        if (!sort && toggle_intake) {
            distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
            ballDetected = distanceCM < distanta_proxsensorCM;

            if (ballDetected && !lastBallDetected && ballCount < 2 && IntakeTimer.milliseconds() > 400) {
                revLeft();   // muta revolver o poziție
                //gamepad2.rumble(600);
                //gamepad1.rumble(600);
                ballCount++;
                IntakeTimer.reset();
                rumbleDone = false;
            }
            if(ballCount==2 && ballDetected && !lastBallDetected && !rumbleDone){
                gamepad1.rumble(1200);
                gamepad2.rumble(1200);
                rumbleDone=true;
            }


            lastBallDetected = ballDetected;
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
                motif_code = id;
                telemetry.addData("Motif Found", id);
                telemetry.update();
                neededOrder[id-21] = 1;
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
    public void revLeft() { ///  din storage in tun
        double err = 0.01;
        double current = rev.getPosition();
    if (Math.abs(current - revpos1) < err) {
        rev.setPosition(revpos2);
    } else if (Math.abs(current - revpos2) < err) {
        rev.setPosition(revpos3);
    } else if (Math.abs(current - revpos3) < err) {
        rev.setPosition(revpos1);
    } else {
        // Default if position doesn't match any
        rev.setPosition(revpos1);
    }

    }

    public void revRight() { ///  din intake in tun
        double err = 0.01;
        double current = rev.getPosition();
        if (Math.abs(current - revpos1) < err) {
            rev.setPosition(revpos3);
        } else if (Math.abs(current - revpos2) < err) {
            rev.setPosition(revpos1);
        } else if (Math.abs(current - revpos3) < err) {
            rev.setPosition(revpos2);
        } else {
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
                push_tun.setPosition(0.1); // retract
                shootTimer.reset();       // reset timer once
                functionshootstate = 1;
                break;

            case 1: // extend after 200ms
                if(shootTimer.milliseconds() > 200){
                    push_tun.setPosition(0.73); // extend
                    functionshootstate = 2;
                }
                break;

            case 2: // done, wait for external reset
                break;
        }
    }


    public int sortState = 0;
    ElapsedTime sort_timer = new ElapsedTime();
    int sortDelay = 350;
    int intun = -1;
    public int [] orderBall = {0 , 0 , 0};
    public boolean readytoshoot = false;
    int rumbleTime = 400;
    boolean secondRevTurn = false;

    public void storage_sort(){
        /*
        toate valorile ajung in array care il folosim la final
        prima minge buna = revLeft
        prima minge rea = rev right
        daca prima minge buna atunci a doua minge revleft
        altfel daca prima minge rea rev right (verificam culoare)
        la final mingea buna in tun // trebuie verificat prin a schimba valorile din orderBall cu culorile, nu daca se alineaza cu motiful
        trebuie verificat daca trebuie rev left sau rev right ca mingea buna sa fie in tun si nustiu cum
        verificam care ii urm buna (ori intake ori storage) si in functie de asta rev right x 2 sau ref left x2
         */
        switch(sortState) {
            case 0:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    //////
                    if (getColor() == neededOrder[0]) {
                        intun = getColor();
                        revLeft();
                        orderBall[0] = 1;
                        sortState++;
                    } else {
                        revRight();
                        orderBall[0] = 0;
                        sortState++;
                    }
                    sort_timer.reset();
                }
                break;
            case 1:
                if (sort_timer.milliseconds() > sortDelay) {
                    sortState++;
                }
                break;
            case 2:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    //////
                    if (getColor() == neededOrder[1]) {
                        orderBall[1] = 1;
                    } else orderBall[1] = 0;
                    if (orderBall[0] == 1) {
                        revLeft();
                        sortState++;
                        // prima bila in tun
                    } else {
                        intun = getColor();
                        revRight();
                        sortState++;
                        // a doua bila in tun
                    }
                    sort_timer.reset();
                }
                break;
            case 3:
                if (sort_timer.milliseconds() > sortDelay) {
                    sortState++;
                }
                break;
            case 4:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    //////
                    gamepad1.rumble(600);
                    gamepad2.rumble(600);
                    if (intun == neededOrder[0]) {
                        //nu se intampla nimic deoarec ii buna
                    }
                    else if (getColor() == neededOrder[0]) { //mingea din intake merge in tun daca ii buna si aia din tun nu
                        revRight();
                        sort_timer.reset();
                    }
                    else { // mingea din storage merge in tun daca celelalte 2 nu sunt bune
                        revLeft();
                        sort_timer.reset();
                    }
                    shootState = 0;
                    sortState++;
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
                            shootTimer.reset();
                        }
                    }
                    break;

                case 1:
                    if (shootTimer.milliseconds() > shootDelay) {
                        if (getColor() == neededOrder[1]) {
                            revRight();
                            secondRevTurn = true;
                        }
                        else {
                            revLeft();
                            secondRevTurn = false;
                        }
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
                            shootTimer.reset();
                        }
                    }
                    break;

                case 3:
                    if (shootTimer.milliseconds() > shootDelay) {
                        if (secondRevTurn) revRight();
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
                            sortState=0;
                            readytoshoot=false;
                        }
                    }
                    break;
            }
        }

    }


    int basic_state=0;
    boolean storage_basic_running = false;
    ElapsedTime timer_storage_basic=new ElapsedTime();

    int revDelay = 600;   // pauza după revLeft (ms)
    int shootDelay = 800; // cât ținem push_tun apăsat (ms)

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
}
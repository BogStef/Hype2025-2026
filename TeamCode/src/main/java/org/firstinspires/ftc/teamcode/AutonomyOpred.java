
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.Pose;import com.pedropathing.paths.PathConstraints;

import java.util.List;


@Autonomous(name = "AUTO Far R - 9 balls", group = "Autonomous")
@Configurable  // Panels
public class AutonomyOpred extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    public ColorSensor colorsensor;
    public DistanceSensor rangeSensor;
    PathConstraints slowConstraints =
            new PathConstraints(0.5, 50, 0.3, 1);

    PathConstraints fastConstraints =
            new PathConstraints(0.9, 100, 0.8, 1);

    public final double revpos1=0.06;
    public final double revpos2=0.45;
    public final double revpos3 = 0.82;

    public DcMotor intake;
    public Limelight3A limelight;

    public Servo push_tun;
    public Servo rev;
    public DcMotorEx tun1;
    public DcMotorEx tun2;
    public double vitTun = 0;
    public double vitTun2 = 0;

    int shootState=-1;
    int motif_code = -1;



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        tun1 = hardwareMap.get(DcMotorEx.class, "tun1");
        tun2 = hardwareMap.get(DcMotorEx.class, "tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun1.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class,"intake");
        push_tun = hardwareMap.get(Servo.class,"push_tun");
        rev = hardwareMap.get(Servo.class,"rev");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(85.57, 14.76, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;

        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");///camera
        limelight.setPollRateHz(120); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);




        tun1.setVelocityPIDFCoefficients(6.7,0,0.1,13.7);
        tun2.setVelocityPIDFCoefficients(6.7,0,0.1,13.7);
        tun1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tun2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");///camera
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);


        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");


        push_tun = hardwareMap.get(Servo.class,"push_tun");
        push_tun.setDirection(Servo.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(81.524, 8.851, Math.toRadians(270)));

        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");

    }
    boolean ballDetected;
    boolean lastBallDetected;
    int ballCount = 0;
    boolean intakeSort = false;
    boolean secondRevLeft = false;
    boolean thirdRevRight = false;
    ElapsedTime IntakeTimer = new ElapsedTime();
    void intake(){
        intake.setPower(.4);
        if (intakeSort) {
            distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
            ballDetected = distanceCM < distanta_proxsensorCM;

            if (ballDetected && !lastBallDetected && ballCount < 3 && IntakeTimer.milliseconds() > 800 && getColor() != -1) {
                if (ballCount == 0) {
                    if (getColor()==2) {
                        revLeft();
                        secondRevLeft = true;
                    }
                    else {
                        revRight();
                        secondRevLeft = false;
                    }
                }
                else if (ballCount == 1) {
                    if (secondRevLeft) {
                        revLeft();
                    }
                    else {
                        if (getColor() == 1) {
                            thirdRevRight = true;
                        }
                        else thirdRevRight = false;
                        revRight();

                    }
                }
                else if (ballCount == 2 && thirdRevRight) revRight();
                ballCount++;
                IntakeTimer.reset();
            }
            lastBallDetected = ballDetected;
        }
    }
    double distanceCM =0;

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing

        pathState = autonomousPathUpdate(); // Update autonomous state machine
        telemetry.addData("motif: ",motif_code);
        telemetry.addData("nrball",nr_ball);
        telemetry.addData("stor 0",stor[0]);
        telemetry.addData("stor 1",stor[1]);
        telemetry.addData("stor 2",stor[2]);
        telemetry.addData("neededorder 0 ",neededOrder[0]);
        telemetry.addData("needeed order 1",neededOrder[1]);
        telemetry.addData("needed order 2",neededOrder[2]);
        telemetry.addData("PathState",pathState);
        telemetry.addData("shootingstate",functionshootstate);

        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
        if (motif_code == -1)
            motif_code = motif(); //cauta motif
        calculateTrueTa();
        double const2 = 3500;
        vitTun = const2;
        vitTun2 = const2+1100;
        if (startingPosition) {
                    rev.setPosition(revpos1);
                    startingPosition=false;
                    push_tun.setPosition(0.75);
                }

        tun(vitTun,vitTun2,true);
        intake();

        switch (motif_code) {
            case 23:
                neededOrder[0] = 1;
                neededOrder[1] = 1;
                neededOrder[2] = 2;
                break;
            case 21: neededOrder[0] = 2;
                neededOrder[1] = 1;
                neededOrder[2] = 1;
                break;
            case 22: neededOrder[0] = 1;
                neededOrder[1] = 2;
                neededOrder[2] = 1;
                break;
            default: ///motif 21
                neededOrder[0] = 2;
                neededOrder[1] = 1;
                neededOrder[2] = 1;

        }
        shoot();



        /// BlueX = RedX
        /// BlueY = 144 - RedY
        /// BlueHeading = -RedHeading
    }

    public int motif() {
        if (motif_code == 21 || motif_code == 22 || motif_code == 23) {
            return motif_code;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return -1;
        }
        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (id == 21 || id == 22 || id == 23) {
                motif_code = id;
                return id;


            }
        }

        return -1;
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()/// poz de tras
                    .addPath(
                            new BezierCurve(
                                    new Pose(81.97,90.77),
                                    new Pose(74,132),
                                    new Pose(100, 100)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(225))
                    .build();

            Path2 = follower.pathBuilder()/// poz pregatire luat bile
                    .addPath(
                            new BezierLine(
                                    new Pose(100, 100),
                                    new Pose(94, 84)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()/// poz luat bile
                    .addPath(
                            new BezierLine(
                                    new Pose(94, 84),
                                    new Pose(128, 84)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()/// poz tras
                    .addPath(
                            new BezierLine(
                                    new Pose(128, 84),
                                    new Pose(100, 100)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                    .build();
            Path5 = follower.pathBuilder().addPath( /// poz preg luat mingi 2
                            new BezierLine(
                                    new Pose(100, 100),

                                    new Pose(87.183, 56.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath( /// luat mingi 2
                            new BezierLine(
                                    new Pose(87.183, 56.500),

                                    new Pose(105.000, 62.100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath( ///// tras
                            new BezierLine(
                                    new Pose(105.000, 62.100),

                                    new Pose(100, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                    .build();
        }
    }
    public double distanta_proxsensorCM=6.7;
    boolean startingPosition = true;
    byte stor[] = {2,1,1}; //0 - tun , 1-intake, 2 - storage ///vmm
    byte shootingPos = 0;
    int nr_ball = 0; ///reset extern
    ElapsedTime bgdtimer = new ElapsedTime();
    boolean storagebgdShooting = false;
    boolean sort_bgd_gata = false;
    int nr_revleft = 0;
    boolean shootanyway = false;

    void sort_bgd_reset(byte a,byte b,byte c){
        storagebgdShooting = false;
        bgdtimer.reset();
        sort_bgd_gata = false;
        nr_revleft = 0;
        shootanyway=false;
        nr_ball =0;
        stor [0] = a;
        stor[1] = b;
        stor[2] = c;
    }
    void sort_bgd(int delay) {

        if (nr_ball >= 3) {
            sort_bgd_gata = true;
            return;
        }
        if (bgdtimer.milliseconds() < delay) return;

        if (stor[0] == 0) {
            revLeft();
            byte temp = stor[0];
            stor[0] = stor[2];
            stor[2] = stor[1];
            stor[1] = temp;
            bgdtimer.reset();
            return;
        }

        // daca culoarea e buna -> trage
        if (stor[0] == neededOrder[nr_ball]||shootanyway) {
            if(shootanyway){
                shootanyway =false;
            }
            if (!storagebgdShooting) {
                storagebgdShooting = true;
                shooting = true;

            }
            if (functionshootstate == 2) {
                nr_ball++;
                stor[0] = 0;
                shooting = false;
                storagebgdShooting = false;
                functionshootstate = 0;
                bgdtimer.reset();
            }
        }
        else if(nr_revleft<=3){/// cre ca trebe <
            revLeft();
            nr_revleft++;
            byte temp = stor[0];
            stor[0] = stor[2];
            stor[2] = stor[1];
            stor[1] = temp;

            bgdtimer.reset();
        }
            else {

        }
    }

    ElapsedTime pathtime = new ElapsedTime();
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: /// pozitie de tras
                follower.followPath(paths.Path1);

                sort_bgd_reset((byte)2,(byte)1,(byte)1);
                pathtime.reset();
                pathState = 1;
                break;
            case 1: ///sorteaza si trage
                if(pathtime.milliseconds()>300&&!follower.isBusy()){
                    if(!sort_bgd_gata){
                        sort_bgd(300);
                        telemetry.addLine("bgdsorting type shi");
                    }
                    if(sort_bgd_gata){
                        pathtime.reset();
                        sort_bgd_gata=false;
                        pathState++;
                    }

                }
                break;

            case 2: ///merge la poz de luat mingi
                if(pathtime.milliseconds()>1200&&!follower.isBusy()){
                    follower.followPath(paths.Path2);
                    ballCount = 0;// reset pt intake
                    IntakeTimer.reset();
                    pathState = 3;
                    pathtime.reset();
                }
                break;
            case 3: /// ia mingi
                if(pathtime.milliseconds()>500&&!follower.isBusy()){
                    follower.followPath(paths.Path3);
                    intakeSort = true;
                    pathState=4;
                    pathtime.reset();

                }
                break;
            case 4: ///merge la pos de tras mingi
                if(pathtime.milliseconds()>2500&&!follower.isBusy()){
                    follower.followPath(paths.Path4);
                    sort_bgd_reset((byte)2,(byte)1,(byte)1);
                    pathState = 5;
                    pathtime.reset();
                }
                break;
            case 5: ///sort si trage
                    if(pathtime.milliseconds()>500&&!follower.isBusy()){
                        intakeSort = false;
                        if(!sort_bgd_gata){
                            sort_bgd(300);
                            telemetry.addLine("bgdsorting type shi");
                        }
                        if(sort_bgd_gata){
                            pathtime.reset();
                            sort_bgd_gata=false;
                            pathState++;
                        }
                    }
                break;
            case 6:
                    if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                        pathState = 7;
                        pathtime.reset();
                    }
                break;
            case 7:/// merge la pozitia de luat mingi 2
                if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                    follower.followPath(paths.Path5);
                    pathState = 8;
                    pathtime.reset();
                }
                break;
            case 8:if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                follower.followPath(paths.Path6);
                pathState = 9;
                pathtime.reset();
            }
                break;
            case 9:if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                follower.followPath(paths.Path7);
                pathState = 10;
                pathtime.reset();
            }
                break;
            case 10:
                break;
        }

        return pathState;
    }

    public int[] neededOrder = {1,1,1};//0 - purple, 1 - green
    public int functionshootstate=0;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean shooting = false;

    public void shoot() {
        if(!shooting) return;

        switch(functionshootstate){
            case 0: // start shoot
                push_tun.setPosition(0); // retract
                shootTimer.reset();       // reset timer once
                functionshootstate = 1;
                break;

            case 1: // extend after 200ms
                if(shootTimer.milliseconds() > 450){
                    push_tun.setPosition(0.75); // extend
                    functionshootstate = 2;
                }
                break;

            case 2: // done, wait for external reset
                break;
        }
    }


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

    public void revRight(){/// din intake in tun
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
    double trueTa = 0;
    public void calculateTrueTa() {
        double ta = 0.00;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            ta = result.getTa();
        }
        else ta=0;
        trueTa=ta; // ne folosim numai de ta pt ca la zalau vom trage numa de pe linie ca urmare unghiul nu ne fut
    }


    void tun(double v1,double v2,boolean toggle_tun) {
        if (toggle_tun) {
            tun1.setVelocity(v1);
            tun2.setVelocity(v2);
        } else {
            tun1.setVelocity(0);
            tun2.setVelocity(0);
        }
    }

    double red,blue,green,alpha;
    public int getColor() { /// 0==mov 1==green
        //double red, blue, green, alpha, error;
        double error;
        error = 120;
        red = colorsensor.red();
        blue = colorsensor.blue();
        green = colorsensor.green();
        alpha = colorsensor.alpha();

        if(green>95&&green>blue&&green>red){
            return 2;
        }
        if(red>green||blue>green)
            return 1;

        return 0;

    }




}

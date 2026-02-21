
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import com.pedropathing.geometry.Pose;

@Autonomous(name = "AUTO Far", group = "Autonomous")
@Configurable // Panels
public class AutonomyOpred extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    public ColorSensor colorsensor;
    public DistanceSensor rangeSensor;

    public final double revpos1=0.00;
    public final double revpos2=0.37;
    public final double revpos3 = 0.76;

    public DcMotor intake;
    public Limelight3A limelight;

    public Servo push_tun;
    public Servo rev;
    public DcMotorEx tun1;
    public DcMotorEx tun2;
    public double vitTun = 0;
    public double vitTun2 = 0;

    int shootState=-1;

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
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);

//        stspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        stft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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


        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");///camera
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);


        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");


        push_tun = hardwareMap.get(Servo.class,"push_tun");
        push_tun.setDirection(Servo.Direction.REVERSE);
        rev = hardwareMap.servo.get("rev");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(85.57, 14.76, 1.49));

        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");


        /*
            sort = !sort;    ///sort in intake
            sortState=0;
            sort_timer.reset();



         */

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);

        calculateTrueTa();
        double const1 = 310, const2 = 900;
        if(trueTa!=0){
            vitTun = const1/trueTa + const2;
            vitTun2 = const1/trueTa +const2+300;
        }
        else {
            vitTun = const2;
            vitTun2 = const2+670;
        }
        tun(vitTun,vitTun2,true);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.57, 14.76),

                                    new Pose(86.58, 17.80)
                            )
                    ).setLinearHeadingInterpolation(1.49, 1.159)

                    .build();
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.58, 17.80),

                                    new Pose(98.360, 42.91)
                            )
                    ).setLinearHeadingInterpolation(1.159, 3.13)

                    .build();
            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(98.360,42.91),

                            new Pose(128.20,41.55)
                    )
            ).setLinearHeadingInterpolation(3.13,3.13)
                    .build();

        }
    }



    ElapsedTime pathtime = new ElapsedTime();
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                pathtime.reset();
                break;

            case 1:
                if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                    follower.followPath(paths.Path2);
                    pathState = 2;
                    pathtime.reset();
                }
                break;
            case 2:
                if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                    follower.followPath(paths.Path3);
                    pathState=3;
                    pathtime.reset();
                }
            case 3:
                break;
        }

        return pathState;
    }

    public int sortState = 0;
    ElapsedTime sort_timer = new ElapsedTime();
    int sortDelay = 350;
    public int [] orderBall = {0 , 0 , 0};
    public boolean readytoshoot = false;

    public double distanceCM=0;
    public double distanta_proxsensorCM=6.7;//real = 13.3

    public int[] neededOrder = {0,0,0};//0 - purple, 1 - green
    boolean storageShooting = false; // new flag to indicate storage-controlled shooting
    public void storage_sort(){
        /*
        toate valorile ajung in array care il folosim la final
        prima minge buna = revLeft
        prima minge rea = rev right
        daca prima minge buna atunci a doua minge revleft
        altfel daca prima minge rea rev right (verificam culoare)
        la final mingea buna in tun
        verificam care ii urm buna (ori intake ori storage) si in functie de asta rev right x 2 sau ref left x2
         */
        switch(sortState) {
            case 0:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    if (getColor() == neededOrder[0]) {
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
            case 1: if (sort_timer.milliseconds() > sortDelay) {
                sortState++;
            }
                break;
            case 2:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    if (getColor() == neededOrder[1]) {
                        orderBall[1] = 1;
                    }
                    else orderBall[1] = 0;
                    if (orderBall[0] == 1) {
                        revLeft();
                        sortState++;
                    } else {
                        revRight();
                        sortState++;
                    }
                    sort_timer.reset();
                }
                break;
            case 3: if (sort_timer.milliseconds() > sortDelay) {
                sortState++;
            }
                break;
            case 4:
                if (distanceCM < distanta_proxsensorCM && getColor() != -1) {
                    if (getColor() == neededOrder[2]) {
                        orderBall[2] = 1;
                    }
                    else orderBall[2] = 0;

                    sortState++;
                }
                break;
            case 5:
                if (orderBall[0] == 0 || orderBall[1] == 0 || orderBall[2] == 0) {
                    if (orderBall[2] == neededOrder[0])
                        revRight();
                }
                sort_timer.reset();
                shootState=0;
                sortState++;
                break;
            case 6:
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
                        if (getColor() == neededOrder[1]) revRight();
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
                        if (getColor() != -1) revRight();
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
                            sortState=0;
                            readytoshoot=false;
                        }
                    }
                    break;
            }
        }

    }
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
                if(shootTimer.milliseconds() > 200){
                    push_tun.setPosition(0.75); // extend
                    functionshootstate = 2;
                }
                break;

            case 2: // done, wait for external reset
                break;
        }
    }


    public int getColor() { /// 0==mov 1==green
        double red, blue, green, alpha;
        double error;
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

        telemetry.addData("vit:",vitTun);
    }




}

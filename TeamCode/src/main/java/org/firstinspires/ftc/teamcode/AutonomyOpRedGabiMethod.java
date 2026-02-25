
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


@Autonomous(name = "AUTO Far R -gabi", group = "Autonomous")
@Configurable  // Panels
public class AutonomyOpRedGabiMethod extends OpMode {
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
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
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
    void intake(){
        intake.setPower(.67);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shoot();
        pathState = autonomousPathUpdate(); // Update autonomous state machine
       /* panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());*/
        telemetry.addData("motif: ",motif_code);
        telemetry.addData("sortstate", sortState);
        //panelsTelemetry.update(telemetry);
        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);
        if (motif_code == -1)
            motif_code = motif(); //cauta motif
        //calculateTrueTa();
        double const1 = 1000, const2 = 850;
        vitTun = const2;
        vitTun2 = const2+1600;

        tun(vitTun,vitTun2,true);
        intake();

        switch (motif_code) {
            case 23: neededOrder[0] = 0;
                neededOrder[1] = 0;
                neededOrder[2] = 1;
                break;
            case 21: neededOrder[0] = 0;
                neededOrder[1] = 1;
                neededOrder[2] = 0;
                break;
            case 22: neededOrder[0] = 1;
                neededOrder[1] = 0;
                neededOrder[2] = 0;
                break;
        }

        if(sort) storage_sort();
        else{}

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
                //telemetry.addData("Motif Found", id);
                //telemetry.update();
                neededOrder[id-21] = 1;
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


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()/// poz de tras
                    .addPath(
                            new BezierLine(
                                    new Pose(81.524, 8.851),
                                    new Pose(85.507, 10)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(253))
                    .build();

            Path2 = follower.pathBuilder()/// poz pregatire luat bile
                    .addPath(
                            new BezierLine(
                                    new Pose(85.507, 10),
                                    new Pose(87.183, 35.795)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(253), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()/// poz luat bile
                    .addPath(
                            new BezierLine(
                                    new Pose(87.183, 35.795),
                                    new Pose(105.069, 41)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()/// poz tras
                    .addPath(
                            new BezierLine(
                                    new Pose(105.069, 41),
                                    new Pose(85.210, 10)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(253))
                    .build();
            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.210,18.090),
                                    new Pose(109.1,36)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(245),Math.toRadians(0))
                    .build();
        }
    }


    ElapsedTime pathtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    boolean sort = false;

    /*
            sort = !sort;
            sortState=0;
            sort_timer.reset();
     */
    void shootaneu(int state,boolean toggle){
        if(!toggle) return;
        ElapsedTime timer = new ElapsedTime();
        switch(state){
            case 0:
                revLeft();
                timer.reset();
                state++;
            case 1:
                if(timer.milliseconds()>200){
                    if (!storageShooting) {
                        storageShooting = true;
                        shooting = true;
                        shoot(); // start shooting
                    }
                    if (functionshootstate == 2) { // shoot finished
                        shooting = false;
                        storageShooting = false;
                        functionshootstate = 0;
                        state++;
                    }
                }
            case 2:
                state = 0;
                toggle = false;
        }



    }
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: /// pozitie de tras
                follower.followPath(paths.Path1);
                sortState = 0;
                sort_timer.reset();
                pathState = 1;
                pathtime.reset();
                break;
            case 1: ///sorteaza si trage
                if(pathtime.milliseconds()>500&&!follower.isBusy()){
                    sort =true;
                    if(pathtime.milliseconds()>1000){
                        readytoshoot = true;
                        pathtime.reset();
                        pathState ++;
                    }
                }
                break;
            case 2: ////verif daca nu cumva s a blocat ceva in intake
                if(pathtime.milliseconds()>5000){
                    if(distanceCM<distanta_proxsensorCM){
                        shootaneu(0,true);
                        pathtime.reset();
                        pathState++;
                    }
                    else pathState++;

                }

            case 3: ///merge la poz de luat mingi
                if(pathtime.milliseconds()>3000&&!follower.isBusy()){
                    follower.followPath(paths.Path2);
                    revLeft();//// aneu
                    pathState ++;
                    pathtime.reset();
                }
                break;
            case 4: /// ia mingi
                if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                    follower.followPath(paths.Path3);
                    sortState = 0; //resetez pt sortarea de la urmatoru path
                    pathState++;
                    pathtime.reset();

                }
                break;
            case 5: ///merge la pos de tras mingi
                if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                    follower.followPath(paths.Path4);
                    pathState ++;
                    pathtime.reset();
                }
                break;
            case 6: ///sorteaza si trage
                if(pathtime.milliseconds()>4000&&!follower.isBusy()){
                        sort =true;
                        if(pathtime.milliseconds()>1000){
                            readytoshoot = true;
                            pathtime.reset();
                            pathState ++;
                        }
                }
                break;
            case 8: ///in caz ca nu le trage pe toate
                if(distanceCM<distanta_proxsensorCM){
                    shootaneu(0,true);
                }
                else pathState++;
            case 9:///park
                if(pathtime.milliseconds()>2000&&!follower.isBusy()){
                    follower.followPath(paths.Path5);
                    pathState ++;
                }
                break;
            case 10:
                break;

        }

        return pathState;
    }


    public int sortState = 0;
    ElapsedTime sort_timer = new ElapsedTime();
    int sortDelay = 350;
    int intun = -1;
    public int [] orderBall = {0 , 0 , 0};
    public boolean readytoshoot = false;
    int rumbleTime = 400;
    boolean secondRevTurn = false;
double distanta_proxsensorCM = 6.7;
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
                    if (getColor() == 0) {
                        //gamepad2.rumble(rumbleTime);
                    } else //gamepad2.rumble(rumbleTime*2);
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
                    if (getColor() == 0) {
                        gamepad2.rumble(rumbleTime);
                    } else gamepad2.rumble(rumbleTime*2);
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
                    if (getColor() == 0) {
                        gamepad2.rumble(rumbleTime);
                    } else gamepad2.rumble(rumbleTime*2);
                    //////
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
                            sort_timer.reset();
                            sortState = 0;
                            readytoshoot = false;

                        }
                    }
                    break;
            }
        }

    }

    double red,blue,green,alpha;

    public double distanceCM=0;


    public int getColor() { /// 0==mov 1==green
        //double red, blue, green, alpha, error;
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


    public int[] neededOrder = {0,0,0};//0 - purple, 1 - green
    boolean storageShooting = false; // new flag to indicate storage-controlled shooting
    int shootDelay = 500;
    //boolean completed = false;
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


    void tun(double v1,double v2,boolean toggle_tun) {
        if (toggle_tun) {
            tun1.setVelocity(v1);
            tun2.setVelocity(v2);
        } else {
            tun1.setVelocity(0);
            tun2.setVelocity(0);
        }
    }




}

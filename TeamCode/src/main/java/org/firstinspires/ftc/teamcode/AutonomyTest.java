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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestCoaie", group = "Autonomous")
@Configurable // Panels
public class AutonomyTest extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private static ElapsedTime timer= new ElapsedTime();
    private static ElapsedTime shootTimer= new ElapsedTime();
    private Servo push_tun,rev;
    private DcMotor tun1,tun2;
    public DistanceSensor rangeSensor;
    private double distanceCM;
    private int nrBallBasic;
    private boolean shooting;
    private double distanta_proxsensorCM;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);


        push_tun = hardwareMap.get(Servo.class,"push_tun");
        rev = hardwareMap.get(Servo.class,"rev");
        tun1 = hardwareMap.get(DcMotor.class,"tun1");
        tun2 = hardwareMap.get(DcMotor.class,"tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setDirection(DcMotor.Direction.REVERSE);


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

        distanceCM = rangeSensor.getDistance(DistanceUnit.CM);//update continuu la distanta
    }
    public void shoot() {
        if(revLeft())
            push_tun.setPosition(1.0);
        shootTimer.reset();
        shooting = true;
    }
    public boolean revLeft(){
        if(rev.getPosition()==0.01){
            rev.setPosition(0.35);
        }
        return true;
    }
    public void revRight(){}
    public void storage_basic(){
        switch(nrBallBasic) {
            case 0:
                if (distanceCM < distanta_proxsensorCM) {
                    revLeft();
                    shoot();
                    nrBallBasic++;
                }
            case 1:
                if (distanceCM < distanta_proxsensorCM) {
                    revLeft();
                    shoot();
                    nrBallBasic++;
                }
            case 2:
                if (distanceCM < distanta_proxsensorCM) {
                    revLeft();
                    shoot();
                    nrBallBasic++;
                }
        }
        if(nrBallBasic>=3) nrBallBasic = 0;
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(105.332, 33.244), new Pose(84.685, 75.237))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.685, 75.237), new Pose(86.960, 125.453))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.960, 125.453), new Pose(105.330, 33.244))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if(timer.time()>1)
                follower.followPath(paths.Path1);
                timer.reset();
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()&&timer.time()>1) {

                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {

                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;

        }
        return pathState;
    }
}

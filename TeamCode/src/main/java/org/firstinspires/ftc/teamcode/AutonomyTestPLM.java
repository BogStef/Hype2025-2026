package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutonomyTestPLM extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private long stateStartTime = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
    }

    public static class Paths {

        public PathChain line1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(80.661, 10.399), new Pose(88, 88))//start to tras
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88, 88), new Pose(88, 83))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88, 83), new Pose(120, 83))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120, 83), new Pose(88, 88))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88, 88), new Pose(88, 62.43))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88, 62), new Pose(120, 62))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120, 62), new Pose(115, 73))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.204, 59.490), new Pose(96.583, 97.283))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.583, 97.283), new Pose(120.029, 71.563))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.line1);
                stateStartTime = System.currentTimeMillis();
                pathState = 100;
                break;

            case 100: // Delay după path 1
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 1;
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 101;
                }
                break;

            case 101: // Delay după path 2
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 102;
                }
                break;

            case 102:
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 103;
                }
                break;

            case 103:
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 104;
                }
                break;

            case 104:
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 105;
                }
                break;

            case 105:
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 106;
                }
                break;

            case 106:
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 9;///STOP
                }
                break;

            case 107:
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    stateStartTime = System.currentTimeMillis();
                    pathState = 108;
                }
                break;

            case 108:
                if (System.currentTimeMillis() - stateStartTime >= 5000) {
                    pathState = 9;
                }
                break;

            case 9:
                telemetry.addLine("gata boss");
                break;
        }
        return pathState;

    }
}

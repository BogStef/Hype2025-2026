
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "AUTO BA", group = "Autonomous")
@Configurable // Panels
public class AutonomyDreapta extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    boolean done = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        pathState = 0;
        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        // Log values to Panels and Driver Station
        pathState = autonomousPathUpdate();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        follower.setStartingPose(new Pose(88.000, 9.000, Math.toRadians(90)));


    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 9.000),

                                    new Pose(88.000, 87.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(225))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 87.500),

                                    new Pose(102.000, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 83.500),

                                    new Pose(124.000, 83.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.000, 83.500),

                                    new Pose(88.000, 87.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 87.500),

                                    new Pose(102.000, 59.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 59.500),

                                    new Pose(124.000, 59.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.000, 59.500),

                                    new Pose(88.000, 87.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 87.500),

                                    new Pose(102.000, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 35.500),

                                    new Pose(124.000, 35.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.000, 35.500),

                                    new Pose(88.000, 87.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 87.500),

                                    new Pose(124.000, 10.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.000, 10.000),

                                    new Pose(133.500, 10.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.500, 10.000),

                                    new Pose(124.500, 25.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.500, 25.000),

                                    new Pose(88.000, 87.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                    .build();
        }
    }

    public int autonomousPathUpdate() {//todo testat
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState++;
                break;
            case 1:
                if (!follower.isBusy()) pathState++;
                break;
            case 2: //shoot
                pathState++;
                break;
            case 3:
                if (!follower.isBusy()) pathState++;
                break;
            case 4:
                follower.followPath(paths.Path2);
                pathState++;
                break;
            case 5:
                if (!follower.isBusy()) pathState++;
                break;
            case 6:
                follower.followPath(paths.Path3);
                pathState++;
                break;
            case 7:
                if (!follower.isBusy()) pathState++;
                break;
            case 8:
                follower.followPath(paths.Path4);
                pathState++;
                break;
            case 9: //shoot
                pathState++;
                break;
            case 10:
                if (!follower.isBusy()) pathState++;
                break;
            case 11:
                follower.followPath(paths.Path5);
                pathState++;
                break;
            case 12:
                if (!follower.isBusy()) pathState++;
                break;
            case 13:
                follower.followPath(paths.Path6);
                pathState++;
                break;
            case 14:
                if (!follower.isBusy()) pathState++;
                break;
            case 15:
                follower.followPath(paths.Path7);
                pathState++;
                break;
            case 16://shoot
                pathState++;
                break;
            case 17:
                if (!follower.isBusy()) pathState++;
                break;
            case 18:
                follower.followPath(paths.Path8);
                pathState++;
                break;
            case 19:
                if (!follower.isBusy()) pathState++;
                break;
            case 20:
                follower.followPath(paths.Path9);
                pathState++;
                break;
            case 21:
                if (!follower.isBusy()) pathState++;
                break;
            case 22:
                follower.followPath(paths.Path10);
                pathState++;
                break;
            case 23://shoot
                pathState++;
                break;
            case 24:
                if (!follower.isBusy()) pathState++;
                break;
            case 25:
                follower.followPath(paths.Path11);
                pathState++;
                break;
            case 26:
                if (!follower.isBusy()) pathState++;
                break;
            case 27:
                follower.followPath(paths.Path12);
                pathState++;
                break;
            case 28:
                if (!follower.isBusy()) pathState++;
                break;
            case 29:
                follower.followPath(paths.Path13);
                pathState++;
                break;
            case 30:
                if (!follower.isBusy()) pathState++;
                break;
            case 31:
                follower.followPath(paths.Path14);
                pathState++;
                break;
            case 32://shoot
                pathState++;
                break;
        }
        return pathState;
    }
}
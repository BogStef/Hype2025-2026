
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
    boolean done=false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        pathState=0;
        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);





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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(59.500, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.500, 83.500),

                                    new Pose(41.000, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.000, 35.500),

                                    new Pose(19.000, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 35.500),

                                    new Pose(57.628, 13.372)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(290))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.628, 13.372),

                                    new Pose(41.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.000, 60.000),

                                    new Pose(19.000, 60.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 60.000),

                                    new Pose(59.500, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.500, 83.500),

                                    new Pose(41.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.000, 84.000),

                                    new Pose(19.000, 84.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 84.000),

                                    new Pose(59.500, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                    .build();
        }
    }
    void auto(){ //todo testat
        switch(pathState){
            case 0: follower.followPath(paths.Path1); //primul path
                pathState++;
                break;
            case 1: if (!follower.isBusy()) pathState++;
                break;
            case 2: //todo shoot
                pathState++;
                break;
            case 3: if (!follower.isBusy()) pathState++;
                break;
            case 4: follower.followPath(paths.Path2); //se duce sa ia mingi pa
                pathState++;
                break;
            case 5: if (!follower.isBusy()) pathState++;
                break;
            case 6: follower.followPath(paths.Path3);
                pathState++;
                break;
            case 7: if (!follower.isBusy()) pathState++;
                break;
            case 8: follower.followPath(paths.Path4);
                pathState++;
                break;
            case 9: //shoot
                pathState++;
                break;
            case 10: if (!follower.isBusy()) pathState++;
                break;
            case 11: follower.followPath(paths.Path5);
                pathState++;
                break;
            case 12: if (!follower.isBusy()) pathState++;
                break;
            case 13: follower.followPath(paths.Path6);
                pathState++;
                break;
            case 14: if (!follower.isBusy()) pathState++;
                break;
            case 15: follower.followPath(paths.Path7);
                pathState++;
                break;
            case 16://shoot
                pathState++;
                break;
            case 17: if (!follower.isBusy()) pathState++;
                break;
            case 18: follower.followPath(paths.Path8);
                pathState++;
                break;
            case 19: if (!follower.isBusy()) pathState++;
                break;
            case 20: follower.followPath(paths.Path9);
                pathState++;
                break;
            case 21: if (!follower.isBusy()) pathState++;
                break;
            case 22: follower.followPath(paths.Path10);
                pathState++;
                break;
            case 23://shoot
                pathState++;
                break;
        }
    }


}
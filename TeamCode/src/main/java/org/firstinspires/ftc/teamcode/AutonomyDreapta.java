
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

@Autonomous(name = "AUTO RED FAR", group = "Autonomous")
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
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 9.000),

                                    new Pose(88.000, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(225))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 87.000),
                                    new Pose(83.170, 51.250),
                                    new Pose(104.000, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(104.000, 35.500),

                                    new Pose(125.750, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.750, 35.500),
                                    new Pose(80.200, 45.700),
                                    new Pose(88, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 87.000),
                                    new Pose(72.000, 30.000),
                                    new Pose(130.500, 8.450)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.500, 8.450),

                                    new Pose(135.75, 8.450)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.75, 8.450),
                                    new Pose(92.500, 42.000),
                                    new Pose(88.000, 87.000)
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

        }
        return 0;

    }
}

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUTO BA", group = "Autonomous")
@Configurable // Panels
public class autonomyStangaBlueClose extends OpMode{
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private org.firstinspires.ftc.teamcode.AutonomyDreapta.Paths paths; // Paths defined in the Paths class
    boolean done = false;

    ElapsedTime shootTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(47.5, 134.5, Math.toRadians(270)));
        pathState = 0;
        paths = new org.firstinspires.ftc.teamcode.AutonomyDreapta.Paths(follower); // Build paths

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
    boolean shooting = false;
    public int functionshootstate=0;
    public Servo push_tun;
    public void shoot() {
        if(!shooting) return;

        switch(functionshootstate){
            case 0:
                shootTimer.reset();
                push_tun.setPosition(.50);
                functionshootstate = 1;
                break;

            case 1:
                if(shootTimer.milliseconds() > 200){
                    push_tun.setPosition(.28);
                    functionshootstate = 2;
                }
                break;

            case 2:
                // gata, așteaptă reset extern
                break;
        }
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
                                    new Pose(47.500, 134.500),

                                    new Pose(36.250, 106.995)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(36.250, 107.000),
                                    new Pose(50.850, 88.000),
                                    new Pose(45.700, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.700, 83.500),

                                    new Pose(18.250, 83.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.250, 83.500),

                                    new Pose(36.250, 107.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(36.250, 107.000),
                                    new Pose(49.000, 80.000),
                                    new Pose(45.700, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.700, 60.000),

                                    new Pose(18.250, 60.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.250, 60.000),

                                    new Pose(36.250, 107.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

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



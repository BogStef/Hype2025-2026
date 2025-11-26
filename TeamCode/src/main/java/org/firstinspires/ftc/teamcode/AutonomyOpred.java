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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
/// Autonomie de miscare doar de test(pozitie intiala=opus de motif, culoarea rosie)
@Autonomous(name = "Autonomy OP - red", group = "Autonomous")
@Configurable
public class AutonomyOpred extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        panelsTelemetry.debug("pathstate", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {/// clasa in care declar toate path urile(sau pathcharinuri in cazul meu)

        public PathChain line1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain line4;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.465, 9.784), new Pose(96.142, 95.929))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.142, 95.929), new Pose(103.374, 35.734))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(103.374, 35.734), new Pose(132.514, 35.947))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.514, 35.947),//start
                                    new Pose(60.000, 40.000),//punct de control
                                    new Pose(96.142, 95.929)//punct de final
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.line1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    /// arunca bilele
                    follower.followPath(paths.Path2);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    ///porneste intake
                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    /// opreste intake
                    follower.followPath(paths.line4);
                    /// arunca bilele
                    pathState = 4;
                }
                break;
        }
        return pathState;
    }
}
///{"startPoint":{"x":81.46528803545053,"y":9.784342688330868,"heading":"linear","startDeg":90,"endDeg":180},"lines":[{"endPoint":{"x":96.14180206794681,"y":95.92909896602659,"heading":"linear","startDeg":90,"endDeg":45},"color":"#898A9C","controlPoints":[]},{"name":"Path 2","endPoint":{"x":103.37370753323486,"y":35.73412112259971,"heading":"linear","reverse":false,"startDeg":45,"endDeg":0},"controlPoints":[],"color":"#CBB878"},{"name":"Path 3","endPoint":{"x":132.51403249630724,"y":35.94682422451994,"heading":"linear","reverse":false,"startDeg":0,"endDeg":0},"controlPoints":[],"color":"#D8D8BA"},{"endPoint":{"x":96.14180206794681,"y":95.92909896602659,"heading":"linear","degrees":0,"startDeg":0,"endDeg":45},"color":"#6ADAB5","controlPoints":[{"x":60,"y":40}]}]}
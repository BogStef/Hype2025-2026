
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "AUTO BUN PT RED", group = "Autonomous")
@Configurable // Panels
public class AutonomyOpred extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime waitTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime shootTimer_f = new ElapsedTime();

    public final double revpos1=0.2;
    public final double revpos2=0.56;
    public final double revpos3 = 0.94;

    public DcMotor intake_up;
    public DcMotor intake_down;

    public Servo push_tun;
    public Servo rev;

    public CRServo improvizatie_intake;
    public boolean pass_intake;
    public DcMotor tun1;
    public DcMotor tun2;
    boolean shooting = false;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        tun1 = hardwareMap.get(DcMotor.class, "tun1");
        tun2 = hardwareMap.get(DcMotor.class, "tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun1.setDirection(DcMotor.Direction.REVERSE);

        intake_up = hardwareMap.get(DcMotor.class, "intake_up");///intake
        intake_down = hardwareMap.get(DcMotor.class, "intake_down");
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_up.setDirection(DcMotor.Direction.REVERSE);
        intake_down.setDirection(DcMotor.Direction.REVERSE);

        push_tun = hardwareMap.get(Servo.class,"push_tun");
        rev = hardwareMap.get(Servo.class,"rev");
        improvizatie_intake = hardwareMap.get(CRServo.class,"improvizatie_intake");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
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

        intake_up.setPower(1);
        intake_down.setPower(1);
        improvizatie_intake.setPower(.6);

        tun1.setPower(1);
        tun2.setPower(1);

        shoot();
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81.546, 9.575),

                                    new Pose(85.045, 22.877)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                    .build();
        }
    }

    public void revLeft() {
        double pos = rev.getPosition();
        double error = 0.01;

        if (Math.abs(pos - revpos1) < error) {
            rev.setPosition(revpos2);
        } else if (Math.abs(pos - revpos2) < error) {
            rev.setPosition(revpos3);
        } else if (Math.abs(pos - revpos3) < error) {
            rev.setPosition(revpos1);
        } else {
            rev.setPosition(revpos1);
        }
    }

    public int shootstate=0;
    public boolean gata_shoot=false;
    public void shoot3bile(){
        switch(shootstate){
            case 0: shooting=true;
                    functionshootstate=0;
                    shootstate++;
                    break;
            case 1: shootTimer.reset();
                    shooting=false;
                    shootstate++;
                    break;
            case 2: if(shootTimer.milliseconds()>1000){
                    revLeft();
                    shootstate++;}
                break;
            case 3:shootTimer.reset();
                    shootstate++;
                    break;
            case 4:if(shootTimer.milliseconds()>1000){
                    shooting=true;
                    functionshootstate=0;
                    shootstate++;
            }
            break;
            case 5:  shootTimer.reset();
                    shooting=false;
                    shootstate++;
                    break;
            case 6: if(shootTimer.milliseconds()>1000){
                    revLeft();
                    shootstate++;
            }
            break;

            case 7:shooting=true;
                    functionshootstate=0;
                    shootTimer.reset();
                    shootstate++;
                    break;
            case 8: if(shootTimer.milliseconds()>1000){
                    shooting=false;
                    gata_shoot=true;
            }



        }
    }

    public int functionshootstate=0;
    public void shoot() {
        if(!shooting) return;

        switch(functionshootstate){
            case 0:
                shootTimer_f.reset();
                push_tun.setPosition(.50);
                functionshootstate = 1;
                break;

            case 1:
                if(shootTimer_f.milliseconds() > 200){
                    push_tun.setPosition(.28);
                    functionshootstate = 2;
                }
                break;

            case 2:
                // gata, așteaptă reset extern
                break;
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1: // Așteaptă să termine Path1
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 2;
                }
                break;

            case 2: // Pauză 1 secundă +shooting
                    if(!gata_shoot)
                        shoot3bile();
                    if(gata_shoot)
                        pathState = 3;

                break;
            case 3:
                // Gata autonomous
                break;
        }

        return pathState;
    }




}

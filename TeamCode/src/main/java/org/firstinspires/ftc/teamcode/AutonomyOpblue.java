package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUTO BUN PT RED", group = "Autonomous")
@Configurable
public class AutonomyOpblue extends OpMode {

    // Telemetrie pentru Panels (debug frumos)
    private TelemetryManager panelsTelemetry;

    // Pedro Pathing follower (ăsta mișcă robotul pe trasee)
    public Follower follower;

    // State-ul principal al autonomiei (ce face robotul acum)
    private int pathState;

    // Obiect care conține toate traseele
    private Paths paths;

    // Timere pentru pauze și shooting
    private ElapsedTime waitTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime shootTimer_f = new ElapsedTime();

    // Poziții posibile pentru servo-ul rev (selectează bilele)
    public final double revpos1 = 0.2;
    public final double revpos2 = 0.56;
    public final double revpos3 = 0.94;

    // Motoare intake
    public DcMotor intake_up;
    public DcMotor intake_down;

    // Servo care împinge bila + servo care mută bilele
    public Servo push_tun;
    public Servo rev;

    // Intake continuu improvizat
    public CRServo improvizatie_intake;

    // Motoarele de la tun
    public DcMotor tun1;
    public DcMotor tun2;

    // Flag care spune dacă trage ACUM
    boolean shooting = false;

    @Override
    public void init() {

        // Inițializare Panels Telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Tun motors
        tun1 = hardwareMap.get(DcMotor.class, "tun1");
        tun2 = hardwareMap.get(DcMotor.class, "tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun1.setDirection(DcMotor.Direction.REVERSE);

        // Intake motors
        intake_up = hardwareMap.get(DcMotor.class, "intake_up");
        intake_down = hardwareMap.get(DcMotor.class, "intake_down");
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_up.setDirection(DcMotor.Direction.REVERSE);
        intake_down.setDirection(DcMotor.Direction.REVERSE);

        // Servo-uri
        push_tun = hardwareMap.get(Servo.class, "push_tun"); // împinge bila
        rev = hardwareMap.get(Servo.class, "rev");           // schimbă bila
        improvizatie_intake = hardwareMap.get(CRServo.class, "improvizatie_intake");

        // Creăm follower-ul Pedro Pathing
        follower = Constants.createFollower(hardwareMap);

        // Poziția de start (robotul CREDE că e aici)
        follower.setStartingPose(new Pose(81.546, 134.425, Math.toRadians(90)));

        // Construim traseele
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        // Pornim autonomia de la state 0
        pathState = 0;
    }

    @Override
    public void loop() {

        // Actualizează poziția și mișcarea robotului pe path
        follower.update();

        // Rulează mașina de stări a autonomiei
        pathState = autonomousPathUpdate();

        // Debug poziție robot
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        // Intake-ul merge NON-STOP
        intake_up.setPower(1);
        intake_down.setPower(1);
        improvizatie_intake.setPower(0.6);

        // Tunurile merg NON-STOP (pregătite să tragă)
        tun1.setPower(1);
        tun2.setPower(1);

        // Dacă shooting == true, împinge bila
        shoot();
    }

    // ======= PATHS =======
    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {

            // Un singur traseu: merge puțin înainte și se aliniază
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(81.546, 134.425), // start
                                    new Pose(85.045, 121.123)  // stop
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(65)
                    )
                    .build();
        }
    }

    // Mută servo-ul rev la următoarea bilă
    public void revLeft() {
        double pos = rev.getPosition();
        double error = 0.01;

        if (Math.abs(pos - revpos1) < error) {
            rev.setPosition(revpos2);
        } else if (Math.abs(pos - revpos2) < error) {
            rev.setPosition(revpos3);
        } else {
            rev.setPosition(revpos1);
        }
    }

    // ======= SHOOT 3 BILE =======
    public int shootstate = 0;
    public boolean gata_shoot = false;

    public void shoot3bile() {
        switch (shootstate) {

            case 0: // pornește prima tragere
                shooting = true;
                functionshootstate = 0;
                shootstate++;
                break;

            case 1: // pauză
                shootTimer.reset();
                shooting = false;
                shootstate++;
                break;

            case 2: // schimbă bila
                if (shootTimer.milliseconds() > 1000) {
                    revLeft();
                    shootstate++;
                }
                break;

            case 3:
                shootTimer.reset();
                shootstate++;
                break;

            case 4: // a doua bilă
                if (shootTimer.milliseconds() > 1000) {
                    shooting = true;
                    functionshootstate = 0;
                    shootstate++;
                }
                break;

            case 5:
                shootTimer.reset();
                shooting = false;
                shootstate++;
                break;

            case 6: // schimbă bila
                if (shootTimer.milliseconds() > 1000) {
                    revLeft();
                    shootstate++;
                }
                break;

            case 7: // a treia bilă
                shooting = true;
                functionshootstate = 0;
                shootTimer.reset();
                shootstate++;
                break;

            case 8: // gata tot shooting-ul
                if (shootTimer.milliseconds() > 1000) {
                    shooting = false;
                    gata_shoot = true;
                }
                break;
        }
    }

    // ======= SHOOT O BILĂ =======
    public int functionshootstate = 0;

    public void shoot() {
        if (!shooting) return;

        switch (functionshootstate) {

            case 0:
                shootTimer_f.reset();
                push_tun.setPosition(0.50); // împinge bila
                functionshootstate = 1;
                break;

            case 1:
                if (shootTimer_f.milliseconds() > 200) {
                    push_tun.setPosition(0.28); // revine
                    functionshootstate = 2;
                }
                break;

            case 2:
                // așteaptă reset extern
                break;
        }
    }

    // ======= AUTONOMOUS STATE MACHINE =======
    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0: // pornește Path1
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1: // așteaptă să termine mersul
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;

            case 2: // trage 3 bile
                if (!gata_shoot)
                    shoot3bile();
                else
                    pathState = 3;
                break;

            case 3:
                // autonomia s-a terminat
                break;
        }
        return pathState;
    }
}
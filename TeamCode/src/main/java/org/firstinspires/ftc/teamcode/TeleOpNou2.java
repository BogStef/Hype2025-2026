

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp - BUN")

public class TeleOpNou2 extends LinearOpMode {
    public DcMotorEx drft, stft, drspa, stspa;
    public DcMotor tun1;
    public DcMotor tun2;
    public IMU imu;
    public ColorSensor colorsensor;
    Limelight3A limelight;
    ElapsedTime timer = new ElapsedTime();

    public boolean pass_intake;
    public boolean toggle_tun;

    public Servo push_back_servo;



    public DcMotor intake_up;
    public DcMotor intake_down;

    public Servo gate;
    public Servo hand1,hand2;



    public final double vmax = 0.7;
    public int motif_code;
    public int getColor() {
        double red,blue,green,alpha,error;
        error=0;
        red = colorsensor.red();
        blue= colorsensor.blue();
        green= colorsensor.green();
        alpha= colorsensor.alpha();
        if     ((red==0+error||red==0-error)&&
                (blue==0+error||blue==0-error)&&
                (green==0+error||green==0-error)&&
                (alpha==0+error||alpha==0-error)){
            return 0;
        }
        if    ((red==1+error||red==1-error)&&
                (blue==1+error||blue==1-error)&&
                (green==0+error||green==0-error)&&
                (alpha==0+error||alpha==0-error)){
            return 1;
        }
        return -1;
    }///TODO - colorsensor test



    @Override
    public void runOpMode()
    {
        initialize();
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while(opModeIsActive())
            {
                // TODO BUG Press dpad_down on controller before dpad_up for the first time
                run();
            }
        }
    }



    public void initialize ()
    {
        stft = hardwareMap.get(DcMotorEx.class, "stft");///miscare
        drft = hardwareMap.get(DcMotorEx.class, "drft");
        stspa = hardwareMap.get(DcMotorEx.class, "stspa");
        drspa = hardwareMap.get(DcMotorEx.class, "drspa");
        stspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drspa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drspa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stspa.setDirection(DcMotor.Direction.REVERSE);
        stft.setDirection(DcMotor.Direction.REVERSE);

        tun1 = hardwareMap.get(DcMotor.class,"tun1");
        tun2= hardwareMap.get(DcMotor.class,"tun2");
        tun1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tun2.setDirection(DcMotor.Direction.REVERSE);

        intake_up = hardwareMap.get(DcMotor.class,"intake_up");///intake
        intake_down = hardwareMap.get(DcMotor.class,"intake_down");
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");///gyro
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP))

        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");///camera
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);

        colorsensor = hardwareMap.get(ColorSensor.class,"colorsensor");

        pass_intake=false;///pass intake
        toggle_tun=false;
        motif_code=-1;

        gate = hardwareMap.get(Servo.class, "gate");
        hand1 = hardwareMap.get(Servo.class, "hand1");
        hand2 = hardwareMap.get(Servo.class, "hand2");


    }

    boolean pressedl = false;
    boolean starerun= true;
    boolean pressedb = false;
    double resunghi=0;
    double unghi=0;
    double maxi = 0;


    double p=25;
    double i=0;
    double d=10;
    double f=36;

    //PID

    public void run()
    {
        boolean isTriggerPressed = gamepad1.cross;
        boolean isTriggerPressedb = gamepad1.circle;

        if(isTriggerPressedb && !pressedb)
        {
            starerun = !starerun;
        }

        if(isTriggerPressed && !pressedl)
        {
            resunghi=-unghi()+180;
        }

        pressedl=isTriggerPressed;
        pressedb=isTriggerPressedb;

        double stftpwr;
        double stspapwr;
        double drftpwr;
        double drspapwr;

        if (timer.seconds() == 1){
            push_back_servo.setPosition(0); //TODO change valeu to act as gate
            gate.setPosition(closed_gate_pos); //not necesary
        }
        if(starerun) {

            /*nghi = unghi() + resunghi;
            if (unghi < 0) {
                unghi = unghi + 360;
            }

            final double x = Math.pow(-gamepad1.left_stick_x, 3.0);
            final double y = Math.pow(gamepad1.left_stick_y, 3.0);

            final double rotation = Math.pow(gamepad1.right_stick_x * 0.8, 3);
            final double direction = Math.atan2(x, y) + Math.toRadians(unghi);
            final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

            stftpwr = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
            drftpwr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
            stspapwr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
            drspapwr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

            double raport=0.657;
            double vit=2787;



            if(gamepad2.right_trigger == 1)
            {
                stft.setVelocity(stftpwr*raport*vit - gamepad2.right_stick_y/4.5);
                drft.setVelocity(drftpwr*raport*vit - gamepad2.right_stick_y/4.5);
                stspa.setVelocity(stspapwr*raport*vit - gamepad2.right_stick_y/4.5);
                drspa.setVelocity(drspapwr*raport*vit - gamepad2.right_stick_y/4.5);
            }else
            {
                stft.setVelocity(stftpwr*raport*vit);
                drft.setVelocity(drftpwr*raport*vit);
                stspa.setVelocity(stspapwr*raport*vit);
                drspa.setVelocity(drspapwr*raport*vit);
            }*/
            double speed;
            if(gamepad2.right_trigger == 1)
            {
                speed = -gamepad1.left_stick_y -gamepad2.right_stick_y/3;
            }else
            {
                speed = -gamepad1.left_stick_y;
            }

            final double turn = gamepad1.right_stick_x;
            final double strafe = gamepad1.left_stick_x;

            stftpwr=speed + turn + strafe;
            drftpwr=speed - turn - strafe;
            stspapwr=speed + turn - strafe;
            drspapwr=speed - turn + strafe;

            double raport=0.585; //experimental
            double vit=2787; //435 rpm / 60 * 345.5 ticks/rev

            stft.setVelocity(stftpwr*raport*vit);
            drft.setVelocity(drftpwr*raport*vit);
            stspa.setVelocity(stspapwr*raport*vit);
            drspa.setVelocity(drspapwr*raport*vit);
            telemetry.update();
        }else {


            stft.setVelocityPIDFCoefficients(p,i,d,f);
            drft.setVelocityPIDFCoefficients(p,i,d,f);
            stspa.setVelocityPIDFCoefficients(p,i,d,f);
            drspa.setVelocityPIDFCoefficients(p,i,d,f);
            //PID

            double speed;
            if(gamepad2.right_trigger == 1)
            {
                speed = -gamepad1.left_stick_y -gamepad2.right_stick_y/3;
            }else
            {
                speed = -gamepad1.left_stick_y;
            }

            final double turn = gamepad1.right_stick_x;
            final double strafe = gamepad1.left_stick_x;

            stftpwr=speed + turn + strafe;
            drftpwr=speed - turn - strafe;
            stspapwr=speed + turn - strafe;
            drspapwr=speed - turn + strafe;

            double raport=0.585; //experimental
            double vit=2787; //435 rpm / 60 * 345.5 ticks/rev

            stft.setVelocity(stftpwr*raport*vit);
            drft.setVelocity(drftpwr*raport*vit);
            stspa.setVelocity(stspapwr*raport*vit);
            drspa.setVelocity(drspapwr*raport*vit);
            telemetry.update();



        }

        if(motif_code==-1)
            motif_code=motif();
        tun(.7);
        intake(.7);


    }

    boolean lastSquare = false;
    void tun(double vit){
        boolean currentSquare = gamepad2.square;
        if (currentSquare && !lastSquare) {
            toggle_tun = !toggle_tun;
        }

        lastSquare = currentSquare;
        if(toggle_tun){
            tun1.setPower(vit);
            tun2.setPower(vit);
        }
        else{
            tun1.setPower(0);
            tun2.setPower(0);
        }
    }

    void intake(double vit){ {}
        if(gamepad2.triangle){
            intake_up.setPower(vit);
            intake_down.setPower(vit);
        }
        if(gamepad2.cross){
            intake_up.setPower(0);
            intake_down.setPower(0);
        }

        if(gamepad2.circle) {
            intake_up.setPower(0);
            pass_intake=!pass_intake;
            if(pass_intake){
                intake_down.setPower(vit);
            }
            else {
                intake_down.setPower(-vit);
            }
        }
    }
    public int motif() {
        if (motif_code == 21 || motif_code == 22 || motif_code == 23) {
            return motif_code;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }
        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return -1;
        }
        for (FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (id == 21 || id == 22 || id == 23) {
                motif_code = id;       // Salvăm motiv-ul definitiv
                telemetry.addData("Motif Found", id);
                telemetry.update();
                return id;
            }
        }

        return -1;
    }
    int caterun=0;

    public double unghi()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        double unghi = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //telemetry.addData(" ", unghi);
        //telemetry.update();
        return unghi+180;
    }

    public void rotation()
    {
        final double rot = gamepad1.right_stick_x;

        drft.setVelocity(2897*-vmax * rot);
        stft.setVelocity(2897*vmax * rot);
        drspa.setVelocity(2897*-vmax * rot);
        stspa.setVelocity(2897*vmax * rot);
    }



    public int green_ball_pos=0;///pozitia bilei verzi: 0 == prima bila care a venit, 1 == a doua, 2 == a treia

    public int degrees_servo_hand1 = 0, degrees_servo_hand2 = 0; ///TODO: set to correct values
    public double max_degrees_servo = 180.0;
    public double pos_servo_hand1 = degrees_servo_hand1/max_degrees_servo;
    public double pos_servo_hand2 = degrees_servo_hand2/max_degrees_servo;

    public double open_gate_pos = 0; // set value //it isnt used anymore
    public double closed_gate_pos = 0; // this one is probably good //it isnt used anymore

    public boolean counting=false;
    public int bile=0;
    public int[] ball_order = {-1, -1};//-1 = gol, 0 = mov, 1 = verde
    public int first_ball = ball_order[0];
    public int second_ball = ball_order[1];
    public double pos_servo_push_back = 0;//todo change value
    public void open_gate(){ //lasa O SINGURA BILA sa treaca si inchide usa dupa
        gate.setPosition(open_gate_pos); //nu mai e necesar
        push_back_servo.setPosition(-pos_servo_push_back);
        timer.reset();
    }
    public void hand_up(){
        if(hand1.getPosition()==pos_servo_hand1)
            if(hand2.getPosition()==pos_servo_hand2)
                System.out.println("ERROR: YOU FUCKING DUMBASS, BOTH HANDS ARE UP!!!!!!!");
            else
                hand2.setPosition(pos_servo_hand2);
        else
            hand1.setPosition(pos_servo_hand1);
    }
    public void hand_down(){
        if(hand1.getPosition()==0)
            if(hand2.getPosition()==0)
                System.out.println("ERROR: YOU FUCKING DUMBASS, BOTH HANDS ARE DOWN!!!!!!!");
            else
                hand2.setPosition(0);
        else
            hand1.setPosition(0);
    }
    /* public void sort() {/// TODO


       switch (motif_code) {
            case 21: // G P P
                switch (first_ball * 10 + second_ball) {
                    case 0:
                        hand_up();
                        hand_up();
                        open_gate();//lasa O SINGURA BILA sa treaca si inchide usa dupa
                        hand_down();
                        open_gate();
                        hand_down();
                        open_gate();
                        break;
                    case 1:
                        hand_up();
                        open_gate();
                        open_gate();
                        hand_down();
                        open_gate();
                        break;
                    case 10:
                        open_gate();
                        open_gate();
                        open_gate();
                        break;
                }
            case 22:// P G P
                switch (first_ball * 10 + second_ball) {
                    case 0:
                        hand_up();
                        open_gate();
                        open_gate();
                        hand_down();
                        open_gate();
                        break;
                    case 1:
                        open_gate();
                        open_gate();
                        open_gate();
                        break;
                    case 10:
                        hand_up();
                        open_gate();
                        hand_down();
                        open_gate();
                        open_gate();
                        break;
                }
            case 23:// P P G
                switch (first_ball * 10 + second_ball) {
                    case 0:
                        open_gate();
                        open_gate();
                        open_gate();
                        break;
                    case 1:
                        open_gate();
                        hand_up();
                        open_gate();
                        hand_down();
                        open_gate();
                        break;
                    case 10:
                        hand_up();
                        open_gate();
                        open_gate();
                        hand_down();
                        open_gate();
                        break;
                }
        }

        */


        public double degrees_revolver_current_pos = 0;
        public Servo hand_rev;
        public void rev_right(){
            degrees_revolver_current_pos += 2/3; // creste cu o treime de rotatie
            if(degrees_revolver_current_pos >= 2) degrees_revolver_current_pos -= 2;
            hand_rev.setPosition(degrees_revolver_current_pos);
        }
        public void rev_left(){
            degrees_revolver_current_pos -= 2/3; // creste cu o treime de rotatie
            if(degrees_revolver_current_pos <= 0) degrees_revolver_current_pos += 2;
            hand_rev.setPosition(degrees_revolver_current_pos);
        }

        public void sort_revolver() {
            switch (motif_code) {
                case 21: // G P P
                    switch (first_ball * 10 + second_ball) {
                        case 0: // p p g
                            rev_right();
                            rev_right();
                            open_gate();
                            rev_left();
                            open_gate();
                            rev_left();
                            open_gate();
                            break;
                        case 1: // P G P
                           rev_right();
                           open_gate();
                           open_gate();
                           rev_left();
                           open_gate();
                           break;
                        case 10: // G P P
                            open_gate();
                            open_gate();
                            open_gate();
                            break;
                    }
                case 22:// P G P
                    switch (first_ball * 10 + second_ball) {
                        case 0: // P P G
                            rev_right();
                            open_gate();
                            open_gate();
                            rev_left();
                            open_gate();
                            break;
                        case 1: // P G P
                            open_gate();
                            open_gate();
                            open_gate();
                            break;
                        case 10: // G P P
                            rev_right();
                            rev_right();
                            open_gate();
                            rev_right();
                            open_gate();
                            rev_right();
                            open_gate();
                            break;
                    }
                case 23:// P P G
                    switch (first_ball * 10 + second_ball) {
                        case 0: // P P G
                            open_gate();
                            open_gate();
                            open_gate();
                            break;
                        case 1: // P G P
                            rev_right();
                            rev_right();
                            open_gate();
                            rev_right();
                            open_gate();
                            rev_right();
                            open_gate();
                            break;
                        case 10: // G P P
                            rev_right();
                            open_gate();
                            open_gate();
                            rev_left();
                            open_gate();
                            break;
                    }
            }
        }

}
public class motor extends LinearOpMode {

   DcMotor motor;
    @Override
    public void runOpMode(){
        initi();
        telemetry.addLine("initialized");
        waitForStart();
        telemetry.update();
        while(opModeIsActive()){
            run();
        }
    }
    public void initi(){
        motor.

    }
    public void run(){

    }
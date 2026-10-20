public class Turret {
     private DcMotorEx rotation_motor;
     private float height=0;
 
     public Servo turret_servo;
     public Servo transfer_1,transfer_2; // dreapta, stanga
     private DcMotorEx launch_motor;
     private double rotation_power = 0.2;
 
     public double gear_ratio = 6.07; // :))

     private int max_left = 135; // pozitiv
     private int max_right = 150; // negativ
 
     private double last_error = 0;
     private com.qualcomm.robotcore.util.ElapsedTime pid_timer = new com.qualcomm.robotcore.util.ElapsedTime();

     private double launch_p, launch_i, launch_d, launch_f;
     private double rotation_p, rotation_i, rotation_d, rotation_f;

     // de la ce vine k?
     public static double kp = 0.022; 
     public static double ki = 0.0;
     public static double kd = 0.0015;

     private boolean last_right_trigger_state = false;
 
     private double turret_servo_position=0;
     private int launch_velocity=1000;

     private double rest_pos_transfer_1 = 0.2;
     private double rest_pos_transfer_2 = 0.4;
     
     private double block_pos_transfer_1 = 0;
     private double block_pos_transfer_2 = 0.54;
 
     public void init(HardwareMap hardware_map) {
          rotation_motor = hardware_map.get(DcMotorEx.class, "rotatie_tun");
          
          launch_motor = hardware_map.get(DcMotorEx.class, "lansare_tun");
          launch_motor.setDirection(DcMotorEx.Direction.FORWARD);
          launch_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
 
          turret_servo = hardware_map.get(Servo.class, "tun_servo");
          turret_servo.setDirection(Servo.Direction.REVERSE);
          turret_servo.setPosition(0);
 

          transfer_1 = hardware_map.get(Servo.class, "transfer1");
          transfer_2 = hardware_map.get(Servo.class,"transfer2");
          transfer_2.setDirection(Servo.Direction.REVERSE);
          transfer_1.setDirection(Servo.Direction.FORWARD);
          transfer_2.setPosition(rest_pos_transfer_2);
 
 
         // FLOAT allows the motor to spin freely when power is 0
         rotation_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
         //rotation_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
         rotation_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
 
         // Use RUN_WITHOUT_ENCODER for the rotation motor so it doesn't
         // try to use internal PID while you are moving it by hand.
         rotation_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
 
         launch_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
 
         launch_p = 43.18;
         launch_i = 1.1;
         launch_d = 0.2;
         launch_f = 13.5;
 
         rotation_p = 0;
         rotation_i = 0;
         rotation_d = 0;
         rotation_f = 0;
         launch_motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(launch_p, launch_i, launch_d, launch_f));
     }
 
     public void run(Gamepad gamepad, Gamepad gamepad2) {
 
         if(gamepad.dpad_right){
               turret_servo_position+=0.005;
         }
 
         if(gamepad.dpad_left){
               turret_servo_position-=0.005;
         }
         if(gamepad.dpad_up){
               launch_velocity+=100;
         }
         if(gamepad.dpad_down){
               launch_velocity-=100;
         }
         if(gamepad.right_bumper){
 
         }
         if (gamepad2.right_trigger>0)
               //rotatie_tun.setPower(0);
               turret_servo_position=Math.max(0.0, Math.min(turret_servo_position, 1.0));
         turret_servo.setPosition(turret_servo_position);
 
         if(gamepad.right_trigger>0.3){
               transfer1.setPosition(rest_pos_transfer1);
               launch_motor.setVelocity(launch_velocity);
               
               transfer_2.setPosition(rest_pos_transfer_2);
               transfer_1.setPosition(rest_pos_transfer_1);
         }
         else{
               launch_motor.setVelocity(0);
               transfer_1.setPosition(block_pos_transfer_1);
               transfer_2.setPosition(block_pos_transfer_2);
         }
     }
 
     public int getLaunchVelocity(){
         return launch_velocity;
     }
 
     public double getServoPosition(){
         return turret_servo_position;
     }
 
     public int getRotation() {
         return rotation_motor.getCurrentPosition();
     }


     public void alignTurretHorizontal(double robot_heading, double robot_x, double robot_y, double target_x, double target_y, Telemetry telemetry) {
          double delta_x = target_x - robot_x;
          double delta_y = target_y - robot_y;
          double field_angle_radians = Math.atan2(delta_y, delta_x);
          double target_relative_radians = field_angle_radians - robot_heading;
          double target_angle_degrees = Math.toDegrees(target_relative_radians);

          double current_angle_degrees = (rotation_motor.getCurrentPosition() / gear_ratio) / ticks_per_degree;

          // Normalize error to [-180, 180] degrees
          double error = target_angle_degrees - current_angle_degrees;
          while (error > 180)  error -= 360;
          while (error <= -180) error += 360;

          // Derivative term with safety guard against division by zero
          double seconds = pid_timer.seconds();
          double derivative = 0;

         if (seconds > 0.0001) {
               derivative = (error - last_error) / seconds;
               pid_timer.reset();
               last_error = error;
         }

          double power = (kp * error) + (kd * derivative);
          double max_power = 0.8;
     
          power = Math.max(-max_power, Math.min(max_power, power));

          // Deadband threshold to eliminate motor chatter near target
          if (Math.abs(error) < 0.8) {
               power = 0;
          }

          // Soft limit enforcement
          if (current_angle_degrees >= max_left && power > 0) {
               power = 0;
          } else if (current_angle_degrees <= (-max_right) && power < 0) {
               power = 0;
          }
         
          if (telemetry != null) {
               telemetry.addData("Turret Target", "%.1f°", target_angle_degrees);
               telemetry.addData("Turret Current", "%.1f°", current_angle_degrees);
               telemetry.addData("Turret Error", "%.1f°", error);
               telemetry.addData("Turret Power", "%.2f", power);
          }

          rotation_motor.setPower(power);
     }
 
     public void alignTurretHorizontal(double robot_heading, double robot_x, double robot_y, Telemetry telemetry) {
          alignTurretHorizontal(robot_heading, robot_x, robot_y, this.target_x, this.target_y, telemetry);
     }
     public void alignTurretVertical(double robot_x, double robot_y){
     }

     public double calculateLaunchVelocity(double distance, double height, double launch_angle_degrees) {
          if (distance <= 0) return 0;
 
          double launch_angle_rad = Math.toRadians(launch_angle_degrees);
          double g = 386.09; // Gravity acceleration in inches/s^2
 
          double tan_theta = Math.tan(launch_angle_rad);
          double cos_theta = Math.cos(launch_angle_rad);
 
          // Denominator in projectile equation: 2 * (d * tan(theta) - h)
          double denominator = 2 * (distance * tan_theta - height);
         
          // Prevent division by zero or imaginary square roots
          if (denominator <= 0.01) {
               denominator = 0.01;
          }

          // Required exit velocity in inches per second
          double required_v0 = (distance / cos_theta) * Math.sqrt(g / denominator);
 
          // Empirical scaling constant converting linear launch velocity (in/s) to motor ticks/s
          double ticks_per_in_sec = 12.5;
          double target_velocity = required_v0 * ticks_per_in_sec;
 
          // Clamp velocity within safe motor operational limits
          double min_velocity = 500.0;
          double max_velocity = 2800.0;
          return Math.max(min_velocity, Math.min(max_velocity, target_velocity));
     }
 
     public double calculateLaunchVelocity(double distance, double height) {
          return calculateLaunchVelocity(distance, height, 35.0);
     }
 
     public void shoot(double distance, double height) {
          double velocity = calculateLaunchVelocity(distance, height);
          this.launch_velocity = (int) Math.round(velocity);
          launch_motor.setVelocity(this.launch_velocity);
     }
 
     public void shoot(float target_distance) {
          shoot(target_distance, this.height);
     }

}
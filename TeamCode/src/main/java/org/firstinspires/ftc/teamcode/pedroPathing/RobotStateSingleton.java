package org.firstinspires.ftc.teamcode.pedroPathing;

public class RobotStateSingleton {
    private static RobotStateSingleton instance = null;

    public enum Alliance { RED, BLUE, NONE }
    private Alliance currentAlliance = Alliance.NONE;

    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    private RobotStateSingleton() {}

    public static RobotStateSingleton getInstance() {
        if (instance == null) {
            instance = new RobotStateSingleton();
        }
        return instance;
    }
    public void setAlliance(Alliance alliance) { this.currentAlliance = alliance; }
    public Alliance getAlliance() { return this.currentAlliance; }

    public void setRobotPose(double x, double y, double heading) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeading = heading;
    }

    public double getX() { return robotX; }
    public double getY() { return robotY; }
    public double getHeading() { return robotHeading; }
}

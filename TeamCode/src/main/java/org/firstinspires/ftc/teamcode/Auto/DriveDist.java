package org.firstinspires.ftc.teamcode.Auto;

public class DriveDist extends AutoCommand {
    private double speed;
    private double dist;
    private double heading;
    private double start;

    public DriveDist(double dist) {
        super();
        this.dist = dist;
        this.speed = 0.75;
        this.heading = 0;

    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
    }
}

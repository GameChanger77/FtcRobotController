package org.firstinspires.ftc.teamcode.odometry;

public class Point {

    public double x, y, r;

    public Point(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
        this.r = 0;
    }

    public Pose toPose(){
        return new Pose(x, y, r);
    }

}

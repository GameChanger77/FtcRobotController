package org.firstinspires.ftc.teamcode.odometry;

public class Line {

    public double x1, x2, y1, y2;

    public Line(double x1, double x2, double y1, double y2) {
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
    }

    public double slope(){
        return (y2 - y1) / (x2 - x1);
    }

    public double yIntercept(){
        return y1 - (slope() * x1);
    }

    public Line toRobotCoords(Pose robotPose){
        double rx = robotPose.getX(), ry = robotPose.getY();
        return new Line (x1 - rx, x2 - rx, y1 - ry, y2 - ry);
    }

    public Line toFieldCoords(Pose robotPose){
        double rx = robotPose.getX(), ry = robotPose.getY();
        return new Line (x1 + rx, x2 + rx, y1 + ry, y2 + ry);
    }
}

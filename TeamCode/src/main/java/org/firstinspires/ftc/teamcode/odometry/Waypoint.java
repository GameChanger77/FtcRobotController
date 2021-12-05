package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;

public class Waypoint extends Pose {

    public double error, power;

    /**
     * The position and orientation of the pose.
     *
     * @param x     Forward component
     * @param y     Rightward component
     * @param theta Clockwise rotation component
     * @param error The radius around the point for error cushion
     * @param power How fast the robot should move toward the point
     */
    public Waypoint(double x, double y, double theta, double error, double power) {
        super(x, y, theta);
        this.error = error;
        this.power = power;
    }

    public void print(Telemetry t){
        super.print(t);
        t.addData("POSE ERROR", error);
        t.addData("POSE POWER", power);
    }

    public void print(GlobalTelemetry t){
        super.print(t);
        t.addData("POSE ERROR", error);
        t.addData("POSE POWER", power);
    }

    public Waypoint getReversedX(){
        return new Waypoint(-x,y,theta,error,power);
    }

    public Pose toPose(){
        return new Pose(x,y,theta);
    }
}

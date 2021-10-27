package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;

public class Pose {

    protected double x, y, theta;

    /**
     * The position and orientation of the pose.
     * @param x Forward component
     * @param y Rightward component
     * @param theta Clockwise rotation component
     */
    public Pose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    /**
     * Prints the current X, Y, and rotation that is stored in the pose.
     * @param t
     */
    public void print(GlobalTelemetry t){
        t.addData("POSE X: ", x);
        t.addData("POSE Y: ", y);
        t.addData("POSE 0: ", theta);
    }

    /**
     * Getters and setters for the components.
     * @return
     */

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void addX(double x){
        this.x += x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void addY(double y){
        this.y += y;
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public void addTheta(double theta){
        this.theta += theta;
    }
}
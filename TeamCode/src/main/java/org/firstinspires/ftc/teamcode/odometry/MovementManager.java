package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

public class MovementManager {

    RobotHardware robot;
    GlobalTelemetry gt;
    OdometryBase gps;

    public MovementManager(RobotHardware robot, GlobalTelemetry gt, OdometryBase gps) {
        this.robot = robot;
        this.gt = gt;
        this.gps = gps;
    }

    public boolean moveToPose(Pose pose, double speed, double error){
        Pose currentPose = gps.getRobotPose();

        double deltaX = pose.getX() - currentPose.getX();
        double deltaY = pose.getY() - currentPose.getY();

        double distance = Math.hypot(deltaX, deltaY);

        if (distance > error){
            double xp = Range.clip(deltaX/distance, -1, 1);
            double yp = Range.clip(deltaY/distance, -1, 1);
            robot.chassis.move(xp, yp, 0, speed);
            return false;
        } else {
            robot.chassis.stop();
            return true;
        }
    }

    public boolean goToPoint(double x, double y, double r, double p, double error){
        Pose currentPose = gps.getRobotPose();

        double deltaX = x - currentPose.getX();
        double deltaY = y - currentPose.getY();

        double distance = Math.hypot(deltaX, deltaY);

        fieldDrive(deltaX,deltaY,r,p);

        if (distance > error){
            return false;
        }

        if(r == 0)
            robot.chassis.stop();
        return true;
    }

    public void fieldDrive(double x, double y, double r, double p){
        double heading = robot.gyro.getHeading();

        double deltaY = getXComponent(heading, y) - getYComponent(heading, x);
        double deltaX = getXComponent(heading, x) + getYComponent(heading, y);

        double yp = Range.clip(deltaY, -1, 1);
        double xp = Range.clip(deltaX, -1, 1);

        robot.chassis.move(xp, yp, r, p);
    }

    /**
     * Gets the forward (y) component of a vector with a certain angle in degrees and a magnitude
     * @param degrees The angle to use
     * @param magnitude The amount of distance traveled at that angle
     * @return The amount of distance traveled along the Y axis
     */
    private double getYComponent(double degrees, double magnitude){
        return Math.sin(Math.toRadians(degrees)) * magnitude;
    }

    /**
     * Gets the rightward (X) component of a vector with a certain angle in degrees and a magnitude
     * @param degrees The angle to use
     * @param magnitude The amount of distance traveled at that angle
     * @return The amount of distance traveled along the X axis
     */
    private double getXComponent(double degrees, double magnitude){
        return Math.cos(Math.toRadians(degrees)) * magnitude;
    }
}

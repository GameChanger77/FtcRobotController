package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

import java.util.ArrayList;

public class OdometryMovement {

    OdometryBase gps;
    RobotHardware robot;

    public double angleError = 1, error = 1;
    double radius = 10,
           kp = 1d/12d, kd = 0,
           lastDistance = 0, lastTime = 0;

    public OdometryMovement(OdometryBase gps, RobotHardware robot) {
        this.gps = gps;
        this.robot = robot;
    }

    public void followPath(ArrayList<Line> path, double power, double angle){

        for (Line line : path){
            Pose robotPose = gps.getRobotPose();

            double deltaX = line.x2 - robotPose.getX();
            double deltaY = line.y2 - robotPose.getY();
            double distance = Math.hypot(deltaX, deltaY);

            while (distance >= radius){
                Point target = findClosestPoint(line.toRobotCoords(robotPose));
                target.r = angle;
                goToPose(target.toPose(), power, error, angleError);
            }
        }

    }

    Point findClosestPoint(Line line){
        double[] xIntercepts = findInterceptX(line, radius);

        double y1 = line.slope() * xIntercepts[0] + line.yIntercept();
        double y2 = line.slope() * xIntercepts[1] + line.yIntercept();

        double dY1 = line.y2 - y1;
        double dX1 = line.x2 - xIntercepts[0];

        double dY2 = line.y2 - y2;
        double dX2 = line.x2 - xIntercepts[1];

        double distance1 = Math.hypot(dX1, dY1);
        double distance2 = Math.hypot(dX2, dY2);

        if(distance1 >= distance2)
            return new Point(xIntercepts[1], y2);

        return new Point(xIntercepts[0], y1);
    }

    double[] findInterceptX(Line line, double radius){
        double m = line.slope();
        double b = line.yIntercept();
        double r2 = radius * radius;

        double a = (-b * b) + r2 + (m * m * r2);

        double x1 = ((-b * m) + Math.sqrt(a) ) /
                (1 + (m * m));

        double x2 = ((-b * m) - Math.sqrt(a) ) /
                (1 + (m * m));

        return new double[] {x1, x2};
    }

    public void goToWaypoint(Waypoint w, long timeAllowed){
        long finalTime = System.currentTimeMillis() + timeAllowed;
        while (goToPose(w.toPose(), w.power, w.error, angleError) && System.currentTimeMillis() <= finalTime){}
    }

    public boolean goToWaypoint(Waypoint w){
        return goToPose(w.toPose(), w.power, w.error, angleError);
    }

    public boolean goToPose(Pose pose, double p, double error, double angleError){
        Pose currentPose = gps.getRobotPose();

        // Field relative triangle
        double deltaX = pose.x - currentPose.getX();
        double deltaY = pose.y - currentPose.getY();
        double deltaW = pose.theta - robot.gyro.getHeading();

        double distance = Math.hypot(deltaX, deltaY);
        double time = System.currentTimeMillis();
        double deltaDistance = lastDistance != 0 ? (distance - lastDistance) : 0;
        double deltaTime = lastTime != 0 ? (time - lastTime) : 0;
        double derivative = deltaDistance / (deltaTime != 0 ? deltaTime : 1);
        double power = Range.clip(Range.clip((distance * kp) + derivative, 0.15, 1) * p, 0.15, 1);

        fieldDrive(deltaX, deltaY, powerToAngle(pose.theta, angleError), power);

        lastDistance = distance;
        lastTime = time;
        if (distance >= error || deltaW >= angleError)
            return true;

        robot.chassis.stop();
        lastDistance = 0;
        lastTime = 0;
        return false;
    }

    /**
     * Drives the robot so that its movement is relative to 0 degrees instead of
     * the front of the robot.
     * @param x Original starboard vector.
     * @param y Original forward vector.
     * @param r Original power clockwise.
     * @param p Scalable power. (Gas pedal)
     */
    public void fieldDrive(double x, double y, double r, double p){
        double heading = robot.gyro.getHeading();

        // Get the robot relative change in x and y
        double deltaX = getCosComponent(heading, x) + getSinComponent(heading, y);
        double deltaY = getCosComponent(heading, y) - getSinComponent(heading, x);

        // Normalize the vectors so we can properly set the powers.
        double distance = Math.hypot(deltaX, deltaY);

        double xp = 0, yp = 0;

        if (distance != 0) {
            xp = deltaX / distance;
            yp = deltaY / distance;
        }

        robot.chassis.move(xp, yp, r, p);
    }

    /**
     * Get the amount of power needed to rotate the robot towards an angle.
     * @param degrees The angle to reach
     * @param error The distance (in degrees) that the robot will stop at. Must be > 0
     * @return The "r" power for other functions to use.
     */
    public double powerToAngle(double degrees, double error){
        double heading =  robot.gyro.getHeading(); // Initial heading
        double distance = degrees - heading; // change in heading

        while (distance > 180) // If the distance > 180 then it is quicker to use the distance - 360
            distance -= 360;
        while (distance < -180)
            distance += 360;

        if (Math.abs(distance) > error)
            return -Range.clip(distance / 90, -1, 1);
        return 0;
    }

    /**
     * Gets the forward (y) component of a vector with a certain angle in degrees and a magnitude
     * @param degrees The angle to use
     * @param magnitude The amount of distance traveled at that angle
     * @return The amount of distance traveled along the Y axis
     */
    private double getSinComponent(double degrees, double magnitude){
        return Math.sin(Math.toRadians(degrees)) * magnitude;
    }

    /**
     * Gets the rightward (X) component of a vector with a certain angle in degrees and a magnitude
     * @param degrees The angle to use
     * @param magnitude The amount of distance traveled at that angle
     * @return The amount of distance traveled along the X axis
     */
    private double getCosComponent(double degrees, double magnitude){
        return Math.cos(Math.toRadians(degrees)) * magnitude;
    }
}

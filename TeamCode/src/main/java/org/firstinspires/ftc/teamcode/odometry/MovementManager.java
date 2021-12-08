package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

public class MovementManager {

    RobotHardware robot;
    GlobalTelemetry gt;
    OdometryBase gps;
    CollisionManager col;
    Telemetry telemetry;

    public double defaultDistanceScale = 5, defaultAngleError = 1.5, defaultAnglePowerScale = 0.75;

    /**
     * USE THIS CONSTRUCTOR IF YOU ARE USING ODOMETRY.
     * @param robot The RobotHardware object
     * @param gt The GlobalTelemetry object
     * @param gps The OdometryBase object
     * @param col The CollisionManager object
     */
    public MovementManager(RobotHardware robot, GlobalTelemetry gt,
                           OdometryBase gps, CollisionManager col, Telemetry telemetry) {
        this.robot = robot;
        this.gt = gt;
        this.gps = gps;
        this.col = col;
        this.telemetry = telemetry;
    }

    public MovementManager(RobotHardware robot, GlobalTelemetry gt,
                           OdometryBase gps, CollisionManager col) {
        this.robot = robot;
        this.gt = gt;
        this.gps = gps;
        this.col = col;
    }

    /**
     * ONLY USE THIS CONSTRUCTOR WHEN YOU ARE NOT USING ODOMETRY.
     * @param robot The RobotHardware object
     * @param gt The GlobalTelemetry object
     */
    public MovementManager(RobotHardware robot, GlobalTelemetry gt) {
        this.robot = robot;
        this.gt = gt;
    }

    /**
     * Makes an arc out of three points. The first point is the robot's position at the start of
     * the path.
     * @param p2 The second point (Vertex of the arc)
     * @param p3 The ending point of the arc
     * @param maximumTimeMillis The maximum time allowed for the robot to attempt the path.
     * @param opMode The running OpMode so we can determine if the opmode is active.
     */
    public void threePointArc(Waypoint p2, Waypoint p3, long maximumTimeMillis, LinearOpMode opMode){
        Pose pose = gps.getRobotPose();
        long startTime = System.currentTimeMillis();
        long finishTime = startTime + maximumTimeMillis;

        Waypoint mid1 = new Waypoint((pose.x + p2.x) / 2,
                0.75 * (p2.y - pose.y) + pose.y,
                p2.theta, p2.error, p2.power);

        Waypoint mid2 = new Waypoint((p2.x + p3.x) / 2, // + p2.x,
                0.25 * (p3.y - p2.y) + p2.y,
                p3.theta, p3.error, p3.power);

        while (goToWaypoint(mid1) && opMode.opModeIsActive()){ // Go to the first mid point
            if (System.currentTimeMillis() > finishTime) {
                telemetry.addData("THREE POINT ARC", "RAN OUT OF TIME ON MID 1!");
                robot.chassis.stop();
                return;
            }
            telemetry.addData("THREE POINT ARC", "RUNNING TO MID 1");
            telemetry.addData("MID X", mid1.x);
            telemetry.addData("MID Y", mid1.y);
            telemetry.update();
        }
        while (goToWaypoint(p2) && opMode.opModeIsActive()){ // go to the second point
            if (System.currentTimeMillis() > finishTime) {
                telemetry.addData("THREE POINT ARC", "RAN OUT OF TIME ON POINT 2!");
                robot.chassis.stop();
                return;
            }
            telemetry.addData("THREE POINT ARC", "RUNNING TO POINT 2");
            telemetry.update();
        }
        while (goToWaypoint(mid2) && opMode.opModeIsActive()){ // go to the second mid point
            if (System.currentTimeMillis() > finishTime) {
                telemetry.addData("THREE POINT ARC", "RAN OUT OF TIME ON MID 2!");
                robot.chassis.stop();
                return;
            }
            telemetry.addData("THREE POINT ARC", "RUNNING TO MID 2");
            telemetry.update();
        }
        while (goToWaypoint(p3) && opMode.opModeIsActive()){ // go to the third point
            if (System.currentTimeMillis() > finishTime) {
                telemetry.addData("THREE POINT ARC", "RAN OUT OF TIME ON POINT 3!");
                robot.chassis.stop();
                return;
            }
            telemetry.addData("THREE POINT ARC", "RUNNING TO POINT 3");
            telemetry.update();
        }

        robot.chassis.stop();
    }

    public boolean goToWaypoint(Waypoint w){
        //return goToPose(w.x, w.y, w.power, w.error, w.theta, 0.5);
        return advancedMove(w.x, w.y, defaultDistanceScale,
                w.power, w.error, w.theta, defaultAngleError, defaultAnglePowerScale);
    }

    public boolean goToWaypoint(Waypoint w, double distanceScale, double angleError){
        return advancedMove(w.x, w.y, distanceScale,
                w.power, w.error, w.theta, angleError, defaultAnglePowerScale);
    }

    public boolean advancedMove(double x, double y, double distanceScale,
                             double p, double error, double degrees, double angleError, double anglePowerScale){
        Pose currentPose = gps.getRobotPose();

        // Field relative triangle
        double deltaX = x - currentPose.getX();
        double deltaY = y - currentPose.getY();
        double distance = Math.hypot(deltaX, deltaY);
        double deltaTheta = degrees - currentPose.theta;


        if (distance >= error || deltaTheta >= angleError) {
            double power = Range.clip(Range.clip(distance / distanceScale, 0.1, 1) * p, 0.1, 1);

            fieldDrive(deltaX, deltaY,
                   (powerToAngle(degrees, angleError) / power) * anglePowerScale,
                       power);
            return true;
        }

        robot.chassis.stop();
        return false;
    }

    /**
     * Make the robot drive towards a certain point relative to the field.
     * This works best as the condition of a while loop because it returns true/false.
     * @param x X coordinate relative to the starting point of the robot.
     * @param y Y coordinate relative to the starting point of the robot.
     * @param r R power to make the robot rotate. (May be replaced with an angle system)
     * @param p P power scaling. (Gas pedal)
     * @param error How close the robot needs to be to a certain point before it will stop.
     * @return True if the robot is moving. False if the robot has reached the point.
     */
    public boolean goToPoint(double x, double y, double r, double p, double error){
        Pose currentPose = gps.getRobotPose();

        // Field relative triangle
        double deltaX = x - currentPose.getX();
        double deltaY = y - currentPose.getY();

        double distance = Math.hypot(deltaX, deltaY);

        fieldDrive(deltaX,deltaY,r,p);

        if (distance > error)
            return true;
        if(r == 0)
            robot.chassis.stop();
        return false;
    }

    /**
     * Make the robot drive towards a certain point relative to the field.
     * This works best as the condition of a while loop because it returns true/false.
     * @param x X coordinate relative to the starting point of the robot.
     * @param y Y coordinate relative to the starting point of the robot.
     * @param p P power scaling. (Gas pedal)
     * @param error How close the robot needs to be to a certain point before it will stop.
     * @param degrees The desired final heading of the robot.
     * @param angleError The amount of allowed error for the final heading of the robot.
     * @return True if the robot is moving. False if the robot has reached the point.
     */
    public boolean goToPose(double x, double y, double p, double error, double degrees, double angleError){
        Pose currentPose = gps.getRobotPose();
        double initialHeading = robot.gyro.getHeading();

        // Field relative triangle
        double deltaX = x - currentPose.getX();
        double deltaY = y - currentPose.getY();
        double deltaW = degrees - initialHeading;

        double distance = Math.hypot(deltaX, deltaY);

        fieldDrive(deltaX, deltaY, powerToAngle(degrees, angleError), p);

        if (distance >= error || deltaW >= angleError)
            return true;
        robot.chassis.stop();
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
        double deltaX = getXComponent(heading, x) + getYComponent(heading, y);
        double deltaY = getXComponent(heading, y) - getYComponent(heading, x);

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
     * Make the robot drive towards a certain point relative to the field.
     * Makes the robot rotate to the desired angle in degrees.
     * This works best as the condition of a while loop because it returns true/false.
     * @param x X coordinate relative to the starting point of the robot.
     * @param y Y coordinate relative to the starting point of the robot.
     * @param degrees The angle to rotate to or stay at.
     * @param p P power scaling. (Gas pedal)
     * @param error How close the robot needs to be to a certain point before it will stop.
     * @return True if the robot is moving. False if the robot has reached the point.
     */
    public boolean goToPose(double x, double y, int degrees, double p, double error){
        return goToPoint(x,y,powerToAngle(degrees, error) * 0.75d,p,error);
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

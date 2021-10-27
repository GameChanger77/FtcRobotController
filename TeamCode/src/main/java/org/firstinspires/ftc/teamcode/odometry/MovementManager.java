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

        double deltaX = x - currentPose.getX();
        double deltaY = y - currentPose.getY();

        double distance = Math.hypot(deltaX, deltaY);

        fieldDrive(deltaX,deltaY,r,p);

        if (distance > error){
            return true;
        }

        if(r == 0)
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

        // Get the field relative change in x and y
        double deltaX = getXComponent(heading, x) + getYComponent(heading, y);
        double deltaY = getXComponent(heading, y) - getYComponent(heading, x);

        // Normalize the vectors so we can properly set the powers.
        double distance = Math.hypot(deltaX, deltaY);

        if (distance != 0) {
            double xp = deltaX / distance;
            double yp = deltaY / distance;

            robot.chassis.move(xp, yp, r, p);
        }

//        double yp = Range.clip(deltaY, -1, 1); // old way
//        double xp = Range.clip(deltaX, -1, 1);
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
     * @param error The distance (in degrees) that the robot will stop at.
     * @return The "r" power for other functions to use.
     */
    public double powerToAngle(double degrees, double error){
        double heading = robot.gyro.getHeading();
        double distance = degrees - heading;

        if (Math.abs(distance) > error){
            if (Math.abs(distance) > 25){
                return Range.clip(distance, -1, 1);
            } else {
                return Range.clip(distance / 5, -1, 1);
            }
        }
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
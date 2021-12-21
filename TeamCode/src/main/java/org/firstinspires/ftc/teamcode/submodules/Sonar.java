package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Point;
import org.firstinspires.ftc.teamcode.odometry.Pose;

import java.util.ArrayList;

public class Sonar {

    // We want to use either the ModernRoboticsI2cRangeSensor or the Rev2mDistanceSensor as the
    // hardware device for the various sensors. Those two have the correct distance units.

    public Rev2mDistanceSensor front;
    public ModernRoboticsI2cRangeSensor right;
    public ModernRoboticsI2cRangeSensor left;
    public Rev2mDistanceSensor back;

    DistanceUnit unit = DistanceUnit.INCH;

    public static final double frontOffset = 4.75,  leftOffset = 3.875 + 2,
                                backOffset = 4.75, rightOffset = 3.875 + 2;

    public void init(HardwareMap hm){
        front = hm.get(Rev2mDistanceSensor.class, "front");
        left = hm.get(ModernRoboticsI2cRangeSensor.class, "left");
        back = hm.get(Rev2mDistanceSensor.class, "back");
        right = hm.get(ModernRoboticsI2cRangeSensor.class, "right");
    }

    public void printDistance(Telemetry telemetry){
        telemetry.addData("front range: ", front.getDistance(unit) + frontOffset);
        telemetry.addData("left  range: ", left.getDistance(unit)  + leftOffset);
        telemetry.addData("back  range: ", back.getDistance(unit)  + backOffset);
        telemetry.addData("right range: ", right.getDistance(unit) + rightOffset);
    }

    public Pose relocatePose(Pose robotPose, double cutoff){
        int i = 0;
        double x = 0, y = 0;

        for (Pose ob : detectObstacles(new Pose(0,0, robotPose.getTheta()), cutoff)) {
            if (ob.getX() < 0 && (ob.getTheta() > 0  && ob.getTheta() < 135)){
                x = -ob.getX();
            } else if (ob.getX() > 0 && (ob.getTheta() < 0  && ob.getTheta() > -135)){
                x = Constants.fieldWidth - ob.getX();
            }

            y = -ob.getY();


            i++;
        }

        return new Pose(x,y,0);
    }

    public Pose relocate(double heading, double cutoff, boolean isRed){
        Pose pose = new Pose(0,0, heading);
        double y = Double.MAX_VALUE;
        double x = 0;

        ArrayList<Pose> obs = detectObstacles(pose, cutoff);

        for (Pose ob : obs){
            if (ob.getY() < y)   // lowest y value
                y = ob.getY();
            if (isRed ? ob.getX() < x : ob.getX() > x)
                x = ob.getX();   // lowest x value for red : greatest x value for blue
        }

        y = -y;                  // negate for correct sign
        x = -x;                  // negate for correct sign

        return new Pose(x, y, heading);
    }

    /**
     * Plots where the detected obstacles are based on the distances and the robot's pose
     * @param robotPose The robots (x, y, 0)
     * @param cutoff how far away do we start ignoring values
     * @return an ArrayList of poses with the obstacles x, y, and O (angle to the object)
     */
    public ArrayList<Pose> detectObstacles(Pose robotPose, double cutoff){
        ArrayList<Pose> obs = new ArrayList<>();
        int i = 0;

        for (double distance : getDistances()){
            if (distance <= cutoff){
                double angle = robotPose.getTheta() + (i * 90);
                double x = -getYComponent(angle, distance) + robotPose.getX();
                double y =  getXComponent(angle, distance) + robotPose.getY();

                obs.add(new Pose(x, y, angle));
            }

            i++;
        }

        return obs;
    }

    /**
     * Returns the distances recorded by the sensors in an array format.
     * @return in this order {front, left, back, right}
     */
    double[] getDistances(){
        return new double[] {front.getDistance(unit) + frontOffset,
                             left.getDistance(unit)  + leftOffset,
                             back.getDistance(unit)  + backOffset,
                             right.getDistance(unit) + rightOffset};
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

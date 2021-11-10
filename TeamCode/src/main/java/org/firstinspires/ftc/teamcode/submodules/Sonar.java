package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Point;
import org.firstinspires.ftc.teamcode.odometry.Pose;

import java.util.ArrayList;

public class Sonar {

    // We want to use either the ModernRoboticsI2cRangeSensor or the SensorREV2mDistance as the
    // hardware device for the various sensors. Those two have the correct distance units.

    public ModernRoboticsI2cRangeSensor front, left, back, right;

    DistanceUnit unit = DistanceUnit.INCH;

    public void init(HardwareMap hm){
        front = hm.get(ModernRoboticsI2cRangeSensor.class, "front");
        left = hm.get(ModernRoboticsI2cRangeSensor.class, "left");
        back = hm.get(ModernRoboticsI2cRangeSensor.class, "back");
        right = hm.get(ModernRoboticsI2cRangeSensor.class, "right");
    }

    public void printDistance(GlobalTelemetry gt){
        gt.addData("front range: ", front.getDistance(unit));
        gt.addData("left range: ", left.getDistance(unit));
        gt.addData("back range: ", back.getDistance(unit));
        gt.addData("right range: ", right.getDistance(unit));
        gt.print();
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
            if (distance < cutoff){
                obs.add(new Pose(getYComponent(robotPose.getTheta(), -distance) + robotPose.getX(),
                                 getXComponent(robotPose.getTheta(),  distance) + robotPose.getY(),
                              robotPose.getTheta() + (i * 90)));
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
        return new double[] {front.getDistance(unit),
                             left.getDistance(unit),
                             back.getDistance(unit),
                             right.getDistance(unit)};
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

package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

import java.util.ArrayList;

public class CollisionManager {

    public ArrayList<Point> obstacles = new ArrayList<>();

    RobotHardware robot;
    GlobalTelemetry gt;
    OdometryBase gps;

    public CollisionManager(RobotHardware robot, GlobalTelemetry gt, OdometryBase gps) {
        this.robot = robot;
        this.gt = gt;
        this.gps = gps;

        init();
    }

    void init(){
        obstacles.add(new Point(0,0,0));
    }


    /**
     * Detects if the point at (X, Y), is outside the boundary of the field.
     * @param x coord of the point
     * @param y coord of the point
     * @return True if outside of the field. False if inside of the field.
     */
    public boolean detectWall(double x, double y){
        Pose pose = gps.getRobotPose();

        if (x > Constants.X_BOUND_HIGH || x < Constants.X_BOUND_LOW ||
            y > Constants.Y_BOUND_HIGH || y < Constants.Y_BOUND_LOW){
            return true;
        }

        return false;
    }

}

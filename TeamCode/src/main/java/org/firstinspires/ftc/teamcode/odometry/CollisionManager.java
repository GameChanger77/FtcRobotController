package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

import java.util.ArrayList;

public class CollisionManager {

    public ArrayList<Point> obstacles = new ArrayList<>();

    RobotHardware robot;
    Telemetry t;
    OdometryBase gps;
    MovementManager move;

    public CollisionManager(RobotHardware robot, Telemetry t, OdometryBase gps, MovementManager move) {
        this.robot = robot;
        this.t = t;
        this.gps = gps;
        this.move = move;

        init();
    }

    void init(){
        obstacles.add(new Point(0,0,0));
    }

    public void enterWarehouseBlue(LinearOpMode opMode){
        Waypoint target = new Waypoint(-115, 1, 0,  2, 0.3);
        Waypoint midstep = new Waypoint(0,0,0,0,0); // to be measured

        ArrayList<Pose> obs = new ArrayList<>();

        while (move.goToWaypoint(midstep, 15, 3) && opMode.opModeIsActive())
            obs = robot.sonar.detectObstacles(gps.getRobotPose(), 30);
    }

    public boolean isObstacleBlockingPath(Pose obs, Pose target) {
        Pose robot = gps.getRobotPose();

        return true;
    }

    /**
     * Detects if the point at (X, Y), is outside the boundary of the field.
     * @param x coord of the point
     * @param y coord of the point
     * @return True if outside of the field. False if inside of the field.
     */
    public boolean detectWall(double x, double y){
        Pose pose = gps.getRobotPose();

        return x > Constants.X_BOUND_HIGH || x < Constants.X_BOUND_LOW ||
                y > Constants.Y_BOUND_HIGH || y < Constants.Y_BOUND_LOW;
    }

}

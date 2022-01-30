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

    /**
     * Detects if there is an obstacle in the path of the robot
     * @param obs The obstacle's pose
     * @param target The target pose
     * @param degreeError The amount of range on either side of the angle to the obstacle
     * @return true if the obstacle is blocking the path false otherwise.
     */
    public boolean isObstacleBlockingPath(Pose obs, Pose target, double degreeError) {
        Pose rPose = gps.getRobotPose();
        double targetDistance = distanceToPoint(rPose, target);
        double obsDistance = distanceToPoint(rPose, target);

        double obsAngle = angleToPoint(rPose, obs);
        double targetAngle = angleToPoint(rPose, target);
        double minAngle = targetAngle - degreeError;
        double maxAngle = targetAngle + degreeError;

        return ((obsDistance <= targetDistance) && // Within distance
                ((obsAngle <= maxAngle) && (obsAngle >= minAngle))); // Within angle
    }

    /**
     * Calculates a new point to avoid an obstacle if the obstacle is blocking the robot's path.
     * If the obstacle is not in the way then it returns the original target position.
     * @param obs The obstacle's pose
     * @param target The target pose
     * @param avoidDistance The amount of distance to pass the obstacle with
     * @return The target point or a new point around the obstacle
     */
    public Pose genericAvoid(Pose obs, Pose target, double avoidDistance){
        if (isObstacleBlockingPath(obs, target, 5)) return target;

        Pose rPose = gps.getRobotPose();

        double targetAngle = angleToPoint(rPose, target);
        double obsAngle = angleToPoint(rPose, obs);

        avoidDistance *= (obsAngle >= targetAngle) ? 1 : -1;

        Pose pose = new Pose(obs.x - Math.cos(obsAngle) * avoidDistance,
                             obs.y + Math.sin(obsAngle) * avoidDistance,
                                target.getTheta());

//        if (detectWall(pose.x, pose.y))
//            pose = new Pose(obs.x - Math.cos(obsAngle) * avoidDistance,
//                            obs.y + Math.sin(obsAngle) * avoidDistance,
//                               target.getTheta());

        return pose;
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

    /**
     * Returns the distance to a point
     * @param start
     * @param target
     * @return
     */
    public double distanceToPoint(Pose start, Pose target){
        return Math.hypot(target.x - start.x, target.y - start.y);
    }

    /**
     * Returns the angle to a point
     * @param start
     * @param target
     * @return
     */
    public double angleToPoint(Pose start, Pose target){
        return Math.toDegrees(Math.atan2(target.y - start.y, target.x - start.x));
    }
}

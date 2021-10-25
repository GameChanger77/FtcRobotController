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
}

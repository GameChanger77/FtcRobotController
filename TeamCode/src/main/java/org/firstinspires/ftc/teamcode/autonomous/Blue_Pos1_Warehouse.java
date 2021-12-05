package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.CollisionManager;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Blue%Pos1%Warehouse%Duck:no", group="Blue")
public class Blue_Pos1_Warehouse extends LinearOpMode {

    Pose startPose = Constants.pos1.getReversedX();

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot, startPose);
    Thread gpsThread = new Thread(gps);
    CollisionManager col = new CollisionManager(robot, gt, gps);
    MovementManager move = new MovementManager(robot, gt, gps, col, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();
//        while (opModeIsActive()) {
//        }
        while (move.goToPose(-115, 0.1, -7, .3, 1) && opModeIsActive()) {
            telemetry.addData("X: ", gps.getRobotPose().getX());
            telemetry.addData("Y: ", gps.getRobotPose().getY());
            telemetry.update();
        }


        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }

}

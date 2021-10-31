package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.CollisionManager;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(group="Test")
public class OdometryTest extends LinearOpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    public RobotHardware robot = new RobotHardware(gt);
    public OdometryBase gps = new OdometryBase(robot, new Pose(0,0,0));
    Thread gpsThread = new Thread(gps);
    CollisionManager col = new CollisionManager(robot, gt, gps);
    MovementManager move = new MovementManager(robot, gt, gps, col);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        // Spinney boi
        while (move.goToPoint(-27, 60, -.5, .75, 1) && opModeIsActive()){
            //gt.print();
        }

        // Spinney boi v2
//        while (opModeIsActive()) {
//            while (move.goToPoint(0, 60, -.75, .75, 1) && opModeIsActive()) {
//            }
//            while (move.goToPoint(60, 60, -.75, .75, 1) && opModeIsActive()) {
//            }
//            while (move.goToPoint(60, 0, -.75, .75, 1) && opModeIsActive()) {
//            }
//            while (move.goToPoint(0, 0, -.75, .75, 1) && opModeIsActive()) {
//            }
//        }

        robot.chassis.stop();
        sleep(200000);

        // Accuracy Test
//        while (move.goToPoint(0, 60, 0, .75, 1)){}
//        while (move.goToPoint(60, 0, 0, .75, 1)){}
//        while (move.goToPoint(0, 0, 0, .75, 1)){}


        robot.chassis.stop();
        gps.stop();
    }

}
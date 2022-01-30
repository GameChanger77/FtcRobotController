package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.CollisionManager;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Point;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(group="Test")
public class OdometryTest extends LinearOpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    public RobotHardware robot = new RobotHardware(gt);
    public OdometryBase gps = new OdometryBase(robot, new Pose(0,0,0));
    Thread gpsThread = new Thread(gps);
    MovementManager move = new MovementManager(robot, gt, gps);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        while (opModeIsActive()){
            Pose target = new Pose(45, 45, 90);
            Pose obstaclePose = target;

            while (move.goToPose(obstaclePose.x, obstaclePose.y, (int) obstaclePose.getTheta(), .3, 1) && opModeIsActive()) {
                while (move.goToPose(target.x, target.y, (int) target.getTheta(), .3, 1) && opModeIsActive()){
                    for (Pose ob: robot.sonar.detectObstacles(gps.getRobotPose(), 50)) {
                        obstaclePose = move.collisionManager.genericAvoid(ob, target, 18);

                        if (!(obstaclePose.x == target.x && obstaclePose.y == target.y && obstaclePose.getTheta() == target.getTheta()))
                            break;
                    }

//                    if (robotpose != obstaclePose)

                    if (!(obstaclePose.x == target.x && obstaclePose.y == target.y && obstaclePose.getTheta() == target.getTheta()))
                        break;
                }

            }

            while (move.goToPose(0, 0, 0, .3, 1) && opModeIsActive()) continue;
        }
        
        
        // Spinney boi
//        while (move.goToPoint(-27, 60, -.5, .75, 1) && opModeIsActive()){
//            //gt.print();
//        }

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

//        robot.chassis.stop();
//        sleep(200000);

        // Accuracy Test
//        while (move.goToPoint(0, 60, 0, .75, 1)){}
//        while (move.goToPoint(60, 0, 0, .75, 1)){}
//        while (move.goToPoint(0, 0, 0, .75, 1)){}


        robot.chassis.stop();
        gps.stop();
    }

}
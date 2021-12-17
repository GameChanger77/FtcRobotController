package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.CollisionManager;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.odometry.Waypoint;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Blue%Pos1%Storage%Duck:yes", group="Blue")
public class Blue_Pos1_Duck_Storage extends LinearOpMode {

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
        move.defaultAnglePower = .2;
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        // Duck Spinner
//        while(move.goToPose(-4, 17, 90, 0.3, 1) && opModeIsActive()){
//            telemetry.update();
//        }
//        while(move.goToPose(-2, 14.5, 90, 0.22, 0.5) && opModeIsActive()){
//            telemetry.update();
//        }
        move.threePointArc(new Waypoint(-8, 20, 0, 2, 0.3),
                           new Waypoint(-2, 15, 90, 0.5, 0.22),
                           15_000L, this);
        robot.chassis.stop();
        //robot.spinner.spinner.setPower(0.25);
        //sleep(5000);
        long finalTime = System.currentTimeMillis() + 5_000;
        while (System.currentTimeMillis() <= finalTime){
            robot.spinner.runAtRPS(2, .25);
            robot.spinner.print(telemetry);
        }

        robot.spinner.spinner.setPower(0);

        // Park
        move.doTimeOut = true;
        move.finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(-10, 20, 0, 0.3, 1) && opModeIsActive()){}
        move.finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(-10, 30, 0.2, 1, 0, 0.5) && opModeIsActive()){
            telemetry.update();
        }

        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }

}

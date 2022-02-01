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

@Autonomous(name="Red%Pos1%Storage%Duck:yes", group="Red")
public class Red_Pos1_Duck_Storage extends LinearOpMode {

    Pose startPose = Constants.pos1;

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot, startPose);
    Thread gpsThread = new Thread(gps);
    MovementManager move = new MovementManager(robot, gt, gps, telemetry);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        robot.spinner.power = 0.1;
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        // Duck Spinner
        long finalTime = System.currentTimeMillis() + 7_000;
        while(move.goToPose(7, 15, 0, 0.3, 1) && opModeIsActive() && System.currentTimeMillis() <= finalTime){telemetry.update();}

        if (!move.withinTarget(new Pose(7, 15, 0), 1)){ // Reset the x pos if needed
            Pose currentPose = gps.getRobotPose();
            gps.overridePosition(new Pose(7, currentPose.y, currentPose.theta));
        }

        finalTime = System.currentTimeMillis() + 7_000;
        while(move.goToPose(8, 9.7, 0, 0.22, 0.5) && opModeIsActive() && System.currentTimeMillis() <= finalTime){ telemetry.update();}
//        move.threePointArc(new Waypoint(-8, 20, 0, 2, 0.3),
//                           new Waypoint(-2, 15, 90, 0.5, 0.22),
//                           15_000L, this);
        robot.chassis.stop();

        finalTime = System.currentTimeMillis() + 10_000;
        while (System.currentTimeMillis() <= finalTime){
            robot.spinner.runAtRPS(-1);
            robot.spinner.print(telemetry);
        }

        robot.spinner.spinner.setPower(0);

        // Park
        move.doTimeOut = true;
        move.finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(10, 20, 0, 0.3, 1) && opModeIsActive()){}
        move.finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(10, 27, 0.2, 1, 0, 0.25) && opModeIsActive()){
            telemetry.update();
        }
        sleep(3000);

        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }
}

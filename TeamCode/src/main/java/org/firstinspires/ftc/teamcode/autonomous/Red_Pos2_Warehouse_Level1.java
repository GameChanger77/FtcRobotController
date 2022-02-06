package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Red%Pos2%Ware%Duck:no%Level1", group="Red")
public class Red_Pos2_Warehouse_Level1 extends LinearOpMode {

    Pose startPose = Constants.pos2;

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
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        move.fieldDrive(-1, 0, 0, 0.23);
        sleep(2_500);

        move.fieldDrive(0,1,0,0.23);
        sleep(2_000);
        robot.chassis.stop();
        while(robot.elevator.level1() && opModeIsActive())
            continue;


        sleep(1000);
        robot.elevator.intake(1);
        sleep(1_500);
        robot.elevator.intake(0);
        sleep(1000);

        // park
        move.fieldDrive(0,-1,0,0.23);
        sleep(2_000);
        robot.chassis.stop();
        while(robot.elevator.level2() && opModeIsActive())
            continue;
        move.fieldDrive(1,0,0,0.2);
        sleep(10_000);
        robot.chassis.stop();


        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }

}

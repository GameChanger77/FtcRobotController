package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Basic Auto Red", group="main")
public class BasicAuto extends LinearOpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    public RobotHardware robot = new RobotHardware(gt);
    public OdometryBase gps = new OdometryBase(robot, new Pose(34, 9,0));
    Thread gpsThread = new Thread(gps);
    MovementManager move = new MovementManager(robot, gt, gps);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        while (move.goToPose(34 + 27, 27, 45, .75, 1)){}
        while (move.goToPose(115, 27, 90, .75, 1)){}


        robot.chassis.stop();
        gps.stop();
    }

}
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

public class BasicAuto extends LinearOpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    public RobotHardware robot = new RobotHardware(gt);
    public OdometryBase gps = new OdometryBase(robot);
    Thread gpsThread = new Thread(gps);
    MovementManager move = new MovementManager(robot, gt, gps);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        while (move.goToPoint(-37, 27, 0, .75, 1)){}


        robot.chassis.stop();
        gps.stop();
    }

}
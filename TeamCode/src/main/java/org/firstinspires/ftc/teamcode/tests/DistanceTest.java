package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;
import org.firstinspires.ftc.teamcode.submodules.Sonar;

@TeleOp
public class DistanceTest extends OpMode {

    Sonar sonar = new Sonar();
    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot);
    Thread gpsThread = new Thread(gps);

    @Override
    public void init() {
        sonar.init(hardwareMap);
    }

    @Override
    public void loop() {
        // sonar.printDistance(telemetry);
        for (Pose obs : sonar.detectObstacles(gps.getRobotPose(), 200)){
            obs.print(gt);
        }

        gt.print();
    }
}

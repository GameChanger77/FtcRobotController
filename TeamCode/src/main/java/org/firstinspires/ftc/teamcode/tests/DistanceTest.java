package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.CollisionManager;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
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
    CollisionManager col = new CollisionManager(robot, gt, gps);
    MovementManager move = new MovementManager(robot, gt, gps, col, telemetry);
    Pose pose;

    @Override
    public void init() {
        sonar.init(hardwareMap);
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gps.showMovement = false;
        gpsThread.start();
        gt.addData("/> STATUS", "INIT COMPLETE");
        gt.print();
    }

    @Override
    public void loop() {
        pose = gps.getRobotPose(); // Get the robot's X, Y, 0

        drive();
        //sonar.printDistance(telemetry);
        for (Pose obs : sonar.detectObstacles(gps.getRobotPose(), 200)){
            obs.print(gt);
        }

        //gt.print();
        //telemetry.update();
    }

    void drive(){
        double power = .25d; // Set the chassis speed
        if(gamepad1.right_trigger > .25)
            power = gamepad1.right_trigger;

        move.goToPoint(pose.getX() + gamepad1.left_stick_x * 2,
                pose.getY() - gamepad1.left_stick_y * 2,
                gamepad1.right_stick_x,
                power, 0.1);

    }

}

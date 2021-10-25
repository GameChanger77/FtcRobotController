package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@TeleOp
public class FieldOrientedDrive extends OpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot);
    Thread gpsThread = new Thread(gps);
    MovementManager move = new MovementManager(robot, gt, gps);

    Pose pose;

    double power = .25d;

    @Override
    public void init() {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        gt.addData("/> STATUS", "INIT COMPLETE");
        gt.print();
    }

    @Override
    public void loop() {
        pose = gps.getRobotPose();

        power = .25d;
        if(gamepad1.right_trigger > .25)
            power = gamepad1.right_trigger;

        move.goToPoint(pose.getX() + gamepad1.left_stick_x * 2,
                pose.getY() - gamepad1.right_stick_y * 2,
                gamepad1.right_stick_x, power, 0.1);

        if (gamepad1.b){
            gps.overridePosition(new Pose(0,0,0));
        }
    }

    @Override
    public void stop() {
        robot.chassis.stop();
        gps.stop();
    }
}

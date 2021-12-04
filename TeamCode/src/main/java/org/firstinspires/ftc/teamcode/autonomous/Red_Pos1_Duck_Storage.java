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
    CollisionManager col = new CollisionManager(robot, gt, gps);
    MovementManager move = new MovementManager(robot, gt, gps, col);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        while (move.goToPose(7, 10, 0, 0.3, 1) && opModeIsActive()) {
        }
        while (move.goToPose(7, 10 - 1.75, 0, 0.2, 0.5) && opModeIsActive()) {
            telemetry.update();
        }
        robot.chassis.stop();
        robot.spinner.spinner.setPower(-0.25);
        sleep(5000);
        robot.spinner.spinner.setPower(0);
        while (move.goToPose(10, 20, 0, 0.3, 1) && opModeIsActive()) {
        }
        while (move.goToPose(10, 26, 0, 0.2, 1) && opModeIsActive()) {
            telemetry.update();
        }

        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }
}

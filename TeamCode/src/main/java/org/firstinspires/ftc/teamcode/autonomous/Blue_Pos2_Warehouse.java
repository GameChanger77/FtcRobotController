package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Blue%Pos2%Warehouse%Duck:no", group="Blue")
public class Blue_Pos2_Warehouse extends LinearOpMode {

    Pose startPose = Constants.pos2.getReversedX();

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

        double finalTime = System.currentTimeMillis() + 5_000;
        while (robot.elevator.level2() && System.currentTimeMillis() <= finalTime && opModeIsActive())
            continue;

        finalTime = System.currentTimeMillis() + 3_000;
        while (move.goToPose(-115, 1, 0, .3, 1) && System.currentTimeMillis() <= finalTime && opModeIsActive()){
            telemetry.addData("X: ", gps.getRobotPose().getX());
            telemetry.addData("Y: ", gps.getRobotPose().getY());
            telemetry.update();
        }


        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }

}

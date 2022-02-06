package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Red%Pos1%Storage%Duck:yes%Level3", group="Red")
public class Red_Pos1_Duck_Storage_level3 extends LinearOpMode {

    Pose startPose = Constants.pos1;

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot, startPose);
    Thread gpsThread = new Thread(gps);
    MovementManager move = new MovementManager(robot, gt, gps, telemetry);

    Paths paths = new Paths(move, this, telemetry, robot, gps);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        robot.spinner.power = 0.1;
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        paths.spinCarouselBlue();

//        move.defaultAnglePower = 0.75;
//        double finalTime = System.currentTimeMillis() + 6_000;
//        while (move.goToPose(gps.getRobotPose().x, gps.getRobotPose().y, 180, 0.23, 1) && System.currentTimeMillis() <= finalTime && opModeIsActive())
//            continue;
//
//        move.defaultAnglePower = 0.75;
//        finalTime = System.currentTimeMillis() + 6_000;
//        while (move.goToPose(55, 23, 180, 0.23, 1) && System.currentTimeMillis() <= finalTime && opModeIsActive())
//            continue;
//
//
//        finalTime = System.currentTimeMillis() + 3_000;
//        while (move.goToPose(55, 23, 0, 0.23, 1) && System.currentTimeMillis() <= finalTime && opModeIsActive())
//            continue;

        double finalTime = System.currentTimeMillis() + 1_500;
        while (System.currentTimeMillis() <= finalTime)
            move.fieldDrive(0, 1, move.powerToAngle(0, 1), 0.23);


        finalTime = System.currentTimeMillis() + 7_500;
        while (System.currentTimeMillis() <= finalTime)
            move.fieldDrive(1, 0, move.powerToAngle(0, 1), 0.23);

        robot.chassis.stop();

        while (robot.elevator.level1() && opModeIsActive())
            continue;

        sleep(1000);

        robot.elevator.intake(1);
        sleep(1_500);
        robot.elevator.intake(0);


        finalTime = System.currentTimeMillis() + 7_500;
        while (System.currentTimeMillis() <= finalTime)
            move.fieldDrive(-1, 0, move.powerToAngle(0, 1), 0.23);

        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }
}

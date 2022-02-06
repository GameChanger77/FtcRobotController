package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Blue%Pos1%Storage%Duck:yes%Level1", group="Blue")
public class Blue_Pos1_Duck_Storage_level1 extends LinearOpMode {

    Pose startPose = new Pose(-24, 0,0); // Constants.pos1.getReversedX();

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
        move.defaultAnglePower = .75;
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        // Duck Spinner
        long finalTime = System.currentTimeMillis() + 5_000;
        while(move.goToPose(-10, 17, 90, 0.3, 1) && opModeIsActive() && System.currentTimeMillis() <= finalTime){telemetry.update();}
        finalTime = System.currentTimeMillis() + 5_000;
        while(move.goToPose(-10, 15, 90, 0.22, 0.5) && opModeIsActive() && System.currentTimeMillis() <= finalTime){ telemetry.update();}

        robot.chassis.stop();

        move.fieldDrive(0,-1,0,.22);
        sleep(4_000);
        robot.chassis.stop();

        finalTime = System.currentTimeMillis() + 5_000;
        while (System.currentTimeMillis() <= finalTime){
            robot.spinner.runAtRPS(1.5);
            robot.spinner.print(telemetry);
        }

        robot.spinner.spinner.setPower(0);

        finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(-10, 15, 0.22, 1,0,1) && System.currentTimeMillis() <= finalTime)
            continue;

        move.fieldDrive(0,1,0,.22);
        sleep(4_000);
        robot.chassis.stop();

        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }

}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Power%Blue%Pos1%Storage%Duck:yes", group="Blue")
public class Power_Blue_Pos1_Storage extends LinearOpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    MovementManager move = new MovementManager(robot, gt);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        move.fieldDrive(1, 1, move.powerToAngle(0, 1), 0.3);
        sleep(2000);
        long finalTime = System.currentTimeMillis() + 5000;
        while (opModeIsActive() && System.currentTimeMillis() <= finalTime){
            robot.chassis.move(0,0,move.powerToAngle(90,1) * 1.25, 0.25);
        }
        move.fieldDrive(1, .5,0, 0.3);
        sleep(2000);
        robot.chassis.stop();
        move.fieldDrive(0, -1, 0, 0.2);
        sleep(2000);
        robot.chassis.stop();
//
//        robot.spinner.spinner.setPower(0.20);
//        sleep(6000);
//        robot.spinner.spinner.setPower(0);

        robot.spinner.runAtRPS(1.5);
        sleep(6000);
        robot.spinner.spinner.setPower(0);

        move.fieldDrive(0, 1, move.powerToAngle(0, 1), 0.2);
        sleep(2000);

        while (opModeIsActive() && getRuntime() < 28){
            robot.chassis.move(0,0,move.powerToAngle(0,1) * 1.25, 0.25);
        }
        robot.chassis.stop();

        robot.sound.playAutoComplete();
    }

}
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Power%Red%Pos1%Storage%Duck:Yes", group="Red")
public class Power_Red_Pos1_Storage extends LinearOpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    MovementManager move = new MovementManager(robot, gt);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        move.fieldDrive(-1, 0.6, move.powerToAngle(0, 1), 0.3);
        sleep(3000);
        move.fieldDrive(-1, 0, move.powerToAngle(0, 1), 0.3);
        sleep(2000);
        move.fieldDrive(0, -1, move.powerToAngle(0, 1), 0.2);
        sleep(1500);
        robot.chassis.stop();

        robot.spinner.spinner.setPower(-0.20);
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
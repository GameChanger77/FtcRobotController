package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Power%Red%Pos1%Storage%Duck:no", group="Red")
public class Power_Red_Pos1_Storage extends LinearOpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    MovementManager move = new MovementManager(robot, gt);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        move.fieldDrive(-1, 1, move.powerToAngle(0, 1), 0.5);
        sleep(2000);

        robot.chassis.stop();
    }

}
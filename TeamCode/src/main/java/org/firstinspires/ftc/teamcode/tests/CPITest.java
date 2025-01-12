package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.CollisionManager;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(group="Test")
public class CPITest extends LinearOpMode {
    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot);
    MovementManager move = new MovementManager(robot, gt, gps);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        telemetry.addData("STATUS", "READY");
        telemetry.update();
        waitForStart();

        gps.printEncoderPositions();
        telemetry.update();
        sleep(3000);
//        robot.chassis.move(1,0,0,.5);
//        sleep(3500);
        while(getRuntime() > 24 && opModeIsActive()){
            robot.chassis.move(1, 0, move.powerToAngle(0, 1), .5);
        }
        robot.chassis.stop();
        gps.printEncoderPositions();
        telemetry.update();
        sleep(20000);
    }


}

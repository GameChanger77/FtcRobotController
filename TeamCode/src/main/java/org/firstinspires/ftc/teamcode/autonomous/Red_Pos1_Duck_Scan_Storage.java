package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(name="Red%Pos1%Storage%Scan:yes%Duck:yes", group="Red")
public class Red_Pos1_Duck_Scan_Storage extends LinearOpMode {

    Pose startPose = Constants.pos1;

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
        robot.spinner.power = 0.1;
        telemetry.addData("/> STATUS:", "INIT COMPLETE");
        waitForStart();

        // 34 inches away (kinda)

        // Duck Spinner
        long finalTime = System.currentTimeMillis() + 10_000;
        while(move.goToPose(10, 15, 0, 0.3, 1) && opModeIsActive() && System.currentTimeMillis() <= finalTime){telemetry.update();}
        while(move.goToPose(10, 9.7, 0, 0.22, 0.5) && opModeIsActive() && System.currentTimeMillis() <= finalTime){ telemetry.update();}

        robot.chassis.stop();

        finalTime = System.currentTimeMillis() + 10_000;
        while (System.currentTimeMillis() <= finalTime){
            robot.spinner.runAtRPS(-1);
            robot.spinner.print(telemetry);
        }

        robot.spinner.spinner.setPower(0);

        // Scan barcode
        move.doTimeOut = true;

        int level = 1; // 1,2, and 3
        double threshold = 34;

        move.finalTime = System.currentTimeMillis() + 3_000;
        while (move.goToPose(35, 20, 0, 0.2, 1)){
            telemetry.addData("/> Distance",
                    robot.sonar.front.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (robot.sonar.front.getDistance(DistanceUnit.INCH) <= threshold) {
                level = 2;
                break;
            }
        }
        move.finalTime = System.currentTimeMillis() + 3_000;
        while (move.goToPose(44, 20, 0, 0.2, 1)){
            telemetry.addData("/> Distance",
                    robot.sonar.front.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (robot.sonar.front.getDistance(DistanceUnit.INCH) <= threshold) {
                level = 3;
                break;
            }
        }

        telemetry.addData("/> LEVEL", level);
        telemetry.update();
        sleep(10_000);

        // Park
        while (move.goToPose(10, 20, 0, 0.3, 1) && opModeIsActive()){}
        move.finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(10, 27, 0.2, 1, 0, 0.25) && opModeIsActive()){
            telemetry.update();
        }
        sleep(3000);

        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }

}

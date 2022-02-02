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

    Paths paths = new Paths(move, this, telemetry, robot, gps);

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
        paths.spinCarouselBlue();

        // Scan barcode
        move.doTimeOut = true;

        int level = 1; // 1,2, and 3
        double threshold = 34;

        move.defaultAnglePower = 1 / 0.2 * .5;
        move.finalTime = System.currentTimeMillis() + 3_000;
        while (move.goToPose(35, 10, 0, 0.2, 1)){
            telemetry.addData("/> Distance", robot.sonar.front.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (robot.sonar.front.getDistance(DistanceUnit.INCH) <= threshold) {
                level = 2;
                break;
            }
        }

        telemetry.addData("/> ", "MIDDLE SCANNED");
        telemetry.update();

        if (level != 2) {
            move.finalTime = System.currentTimeMillis() + 3_000;
            while (move.goToPose(44, 10, 0, 0.2, 1)) {
                telemetry.addData("/> Distance", robot.sonar.front.getDistance(DistanceUnit.INCH));
                telemetry.update();
                if (robot.sonar.front.getDistance(DistanceUnit.INCH) <= threshold) {
                    level = 3;
                    break;
                }
            }
        }

        telemetry.addData("/> LEVEL", level);
        telemetry.update();
        sleep(1000);
        while (move.goToPose(60, 10, 0, 0.2, 1)){ telemetry.update(); }

        telemetry.addData("/> LEVEL", level);
        telemetry.update();
        sleep(5_000);

        // Place in correct level
        paths.placeBlock(level);

        // Park
        paths.parkInStorageBlue();

        robot.chassis.stop();
        gps.stop();

        robot.sound.playAutoComplete();
    }

}

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

public class Paths {

    private MovementManager move;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private RobotHardware robot;
    private OdometryBase gps;

    public Paths(MovementManager move, LinearOpMode opMode, Telemetry telemetry, RobotHardware robot, OdometryBase gps) {
        this.move = move;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.robot = robot;
        this.gps = gps;
    }

    public void placeBlock(int level){
        move.doTimeOut = false;
        // Move towards alliance hub (may need to change x and y)
        long finalTime = System.currentTimeMillis() + 5_000;
        while(move.goToPose(30, 20, 0, 0.22, 1.5) && opMode.opModeIsActive() && System.currentTimeMillis() <= finalTime){
            telemetry.update();
        }
        robot.chassis.stop();

        // Move lift to correct level
        switch(level){
            case 1:
                while (robot.elevator.level1() && opMode.opModeIsActive()){ telemetry.update(); }
                break;
            case 2:
                while (robot.elevator.level2() && opMode.opModeIsActive()){ telemetry.update(); }
                break;
            case 3:
                while (robot.elevator.level3() && opMode.opModeIsActive()){ telemetry.update(); }
                break;
        }

        // Spit out cube
        robot.elevator.intake(1); // Might need to be negative
        opMode.sleep(1_000);
        robot.elevator.intake(0);

        // Back away from alliance hub (may need to change x and y)
        while(move.goToPose(30, 15, 0, 0.22, 1.5) && opMode.opModeIsActive() && System.currentTimeMillis() <= finalTime){
            telemetry.update();
        }
    }

    public void spinCarouselBlue(){
        // Move toward the carousel
        move.doTimeOut = false;
        long finalTime = System.currentTimeMillis() + 7_000;
        while(move.goToPose(10, 15, 0, 0.3, 1) && opMode.opModeIsActive() && System.currentTimeMillis() <= finalTime){telemetry.update();}

        if (!move.withinTarget(new Pose(10, 15, 0), 1)){ // Reset the x pos if needed
            Pose currentPose = gps.getRobotPose();
            gps.overridePosition(new Pose(10, currentPose.y, currentPose.theta));
        }

        finalTime = System.currentTimeMillis() + 5_000;
        while(move.goToPose(10, 9.7, 0, 0.22, 0.5) && opMode.opModeIsActive() && System.currentTimeMillis() <= finalTime){ telemetry.update();}

        finalTime = System.currentTimeMillis() + 1_000;
        while(move.goToPose(10, 5, 0, 0.22, 0.5) && opMode.opModeIsActive() && System.currentTimeMillis() <= finalTime){ telemetry.update();}
        robot.chassis.stop();

        // Spin the carousel
        finalTime = System.currentTimeMillis() + 5_000;
        while (System.currentTimeMillis() <= finalTime){
            robot.spinner.runAtRPS(-1.25);
            robot.spinner.print(telemetry);
        }

        robot.spinner.spinner.setPower(0);
    }

    public void parkInStorageBlue(){
        // Park
        move.doTimeOut = true;
        move.finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(10, 20, 0, 0.3, 1) && opMode.opModeIsActive()){}
        robot.elevator.up();
        move.finalTime = System.currentTimeMillis() + 5_000;
        while (move.goToPose(10, 21.5, 0.2, 1, 0, 0.25) && opMode.opModeIsActive()){
            telemetry.update();
        }
        opMode.sleep(3000);
        robot.chassis.stop();
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@TeleOp(name="Test Chassis", group="test")
public class TestChassis extends OpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot);
    Thread gpsThread = new Thread(gps);

    double power = .25d;

    @Override
    public void init() {
        robot.init(hardwareMap);
        gps.init(hardwareMap);
        gpsThread.start();
        gt.addData("/> STATUS", "INIT COMPLETE");
        gt.print();
    }

    @Override
    public void loop() {
        power = .25d;
        if(gamepad1.right_trigger > .25)
            power = gamepad1.right_trigger;

        if(gamepad1.x)
            robot.chassis.burnout(gamepad1.right_trigger);
        else if(gamepad1.y) {
            robot.chassis.forward();
        }else{
            robot.chassis.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, power);
        }

        if (gamepad1.b){
            gps.overridePosition(new Pose(0,0,0));
        }


//
//        gt.addData("/> STATUS", "OK");
//        gt.addData("/> Raw Heading", robot.gyro.getHeading());
//        gt.addData("/> ", "Heading: " + robot.gyro.getHeading() * 2);
//        gt.addData("/> ", "Angle: " + (90 - robot.gyro.getHeading() * 2))
        //gt.print();
    }

    @Override
    public void stop() {
        robot.chassis.stop();
        gps.stop();
    }
}

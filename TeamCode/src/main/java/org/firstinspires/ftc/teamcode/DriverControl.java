package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.CollisionManager;
import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@TeleOp(name="Driver Control (Field)", group="main")
public class DriverControl extends OpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot);
    Thread gpsThread = new Thread(gps);
    CollisionManager col = new CollisionManager(robot, gt, gps);
    MovementManager move = new MovementManager(robot, gt, gps, col);

    Pose pose;

    double power = .25d;

    boolean fieldDrive = true;

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
        pose = gps.getRobotPose(); // Get the robot's X, Y, Z

        power = .25d; // Set the chassis speed
        if(gamepad1.right_trigger > .25)
            power = gamepad1.right_trigger;

        if (fieldDrive) // Make the chassis move relative to the field.
            move.goToPoint(pose.getX() + gamepad1.left_stick_x * 2,
                pose.getY() - gamepad1.left_stick_y * 2,
                gamepad1.right_stick_x, power, 0.1);
        else // Traditional drive forward relative to robot
            robot.chassis.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, power);

        if(gamepad1.x) // Burnout
            robot.chassis.burnout(gamepad1.right_trigger);

        if(gamepad1.dpad_up)
            fieldDrive = true;
        if(gamepad1.dpad_down && gamepad1.x && gamepad1.right_bumper && gamepad1.left_bumper)
            fieldDrive = false;

        // Non-Driving functions
        robot.conveyor.motor.setPower(-gamepad2.right_stick_y);
        robot.spinner.spinner.setPower(-gamepad2.left_stick_y/2);

        // Reset the pose to the origin.
        if (gamepad1.b){
            gps.overridePosition(new Pose(0,0, robot.gyro.getHeading()));
        }

    }

    @Override
    public void stop() {
        robot.chassis.stop();
        gps.stop();
    }
}

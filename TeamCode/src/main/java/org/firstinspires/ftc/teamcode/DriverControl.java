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
    int alignAngle = 0;

    // Settings
    boolean fieldDrive = true;
    boolean wasDpUp = false, wasDpRight = false, wasDpDown = false, wasDpLeft = false,
            wasRJdown = false, autoAlign = false;
    boolean trainingWheels = false;

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
        pose = gps.getRobotPose(); // Get the robot's X, Y, 0
        autoAlign();
        drive();

        // Non-Driving functions
        robot.conveyor.motor.setPower(-gamepad2.right_stick_y);
        robot.spinner.spinner.setPower(-gamepad2.left_stick_y/2);

        // Reset the pose to the origin.
        if (gamepad1.b)
            gps.overridePosition(new Pose(0,0, robot.gyro.getHeading()));

        telemetry.update();
    }

    void autoAlign(){
        boolean isDpUp = gamepad1.dpad_up;
        boolean isDpRight = gamepad1.dpad_right;
        boolean isDpDown = gamepad1.dpad_down;
        boolean isDpLeft = gamepad1.dpad_left;
        boolean isRJdown = gamepad1.left_stick_button;

        if(isRJdown && !wasRJdown)
            autoAlign = false;

        if(isDpUp && !wasDpUp){
            autoAlign = true;
            alignAngle = 0;
            robot.sound.playAutoAlign();
        } else if(isDpRight && !wasDpRight){
            autoAlign = true;
            alignAngle = -90;
            robot.sound.playAutoAlign();
        } else if(isDpDown && !wasDpDown){
            autoAlign = true;
            alignAngle = 180;
            robot.sound.playAutoAlign();
        } else if(isDpLeft && !wasDpLeft){
            autoAlign = true;
            alignAngle = 90;
            robot.sound.playAutoAlign();
        }

        telemetry.addData("Alignment", autoAlign ? "AUTO @" + alignAngle : "MANUAL @" + robot.gyro.getHeading());
    }

    void drive(){
        power = .25d; // Set the chassis speed
        if(gamepad1.right_trigger > .25)
            power = gamepad1.right_trigger * (trainingWheels ? 0.3 : 1);

        if (fieldDrive) // Make the chassis move relative to the field.
            move.goToPoint(pose.getX() + gamepad1.left_stick_x * 2,
                    pose.getY() - gamepad1.left_stick_y * 2,
                    autoAlign ? move.powerToAngle(alignAngle, 1) : gamepad1.right_stick_x,
                    power, 0.1);
        else // Traditional drive forward relative to robot
            robot.chassis.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, power);

        if(gamepad1.x && !trainingWheels) // Burnout
            robot.chassis.burnout(power);

        // Toggle Field/headless Drive and Robot Drive
        if(gamepad1.dpad_up)
            fieldDrive = true;
        if(gamepad1.dpad_down && gamepad1.x && gamepad1.right_bumper && gamepad1.left_bumper){
            fieldDrive = false;
            robot.sound.playAdminOverride();
        }

        // Training wheels activation and deactivation
        if(gamepad1.dpad_right && gamepad1.x && gamepad1.right_bumper && gamepad1.left_bumper && !wasDpRight) {
            trainingWheels = true;
            robot.sound.playTrainingWheels();
        }

        if (gamepad1.dpad_left && gamepad1.x && gamepad1.right_bumper && gamepad1.left_bumper && !wasDpLeft){
            trainingWheels = false;
            robot.sound.playAdminOverride();
        }

    }

    @Override
    public void stop() {
        robot.chassis.stop();
        gps.stop();
    }
}

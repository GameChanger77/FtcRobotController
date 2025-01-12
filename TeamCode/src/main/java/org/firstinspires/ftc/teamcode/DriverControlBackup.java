package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.MovementManager;
import org.firstinspires.ftc.teamcode.odometry.OdometryBase;
import org.firstinspires.ftc.teamcode.odometry.Pose;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@TeleOp(name="Backup Driver Control", group="main")
public class DriverControlBackup extends OpMode {

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    OdometryBase gps = new OdometryBase(robot);
    Thread gpsThread = new Thread(gps);
    MovementManager move = new MovementManager(robot, gt, gps, telemetry);

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
        robot.elevator.lift(gamepad1.left_trigger - gamepad1.right_trigger);
        if (gamepad1.x) robot.elevator.pickup();
        if (gamepad1.b) robot.elevator.hold();

        robot.elevator.intake(gamepad1.left_bumper ? -1 :
                gamepad1.right_bumper ? 1 : 0);

        robot.spinner.print(telemetry);
        telemetry.addData("/> Elevator", robot.elevator.lift.getCurrentPosition());

        robot.spinner.print(telemetry);
        if (gamepad1.y || gamepad2.y) robot.spinner.runAtRPS(1.5); //  Duck spinner test
        else robot.spinner.spinner.setPower(-gamepad2.left_stick_y/4);

        // Reset the pose to the origin.
        if (gamepad1.b)
            gps.overridePosition(new Pose(0,0, robot.gyro.getHeading()));

        // telemetry.update();
    }

    void autoAlign(){
        boolean isDpUp = gamepad1.dpad_up;
        boolean isDpRight = gamepad1.dpad_right;
        boolean isDpDown = gamepad1.dpad_down;
        boolean isDpLeft = gamepad1.dpad_left;
        boolean isRJdown = -gamepad1.right_stick_y > 0.5 ? true : false;

        if(isRJdown && !wasRJdown)
            autoAlign = false;

        if(isDpUp && !wasDpUp && !gamepad1.x){
            autoAlign = true;
            alignAngle = 0;
            robot.sound.playAutoAlign();
        } else if(isDpRight && !wasDpRight && !gamepad1.x){
            autoAlign = true;
            alignAngle = -90;
            robot.sound.playAutoAlign();
        } else if(isDpDown && !wasDpDown && !gamepad1.x){
            autoAlign = true;
            alignAngle = 180;
            robot.sound.playAutoAlign();
        } else if(isDpLeft && !wasDpLeft && !gamepad1.x){
            autoAlign = true;
            alignAngle = 90;
            robot.sound.playAutoAlign();
        }

        wasDpUp = isDpUp;
        wasDpRight = isDpRight;
        wasDpDown = isDpDown;
        wasDpLeft = isDpLeft;
        wasRJdown = isRJdown;

        telemetry.addData("Alignment", autoAlign ? "AUTO @" + alignAngle : "MANUAL @" + robot.gyro.getHeading());
    }

    void drive(){
        power = .25d; // Set the chassis speed
        if(gamepad1.right_trigger > .25)
            power = gamepad1.right_trigger * (trainingWheels ? 0.3 : 1);

        if (fieldDrive) // Make the chassis move relative to the field.
            move.goToPoint(pose.getX() + gamepad1.left_stick_x * 2,
                    pose.getY() - gamepad1.left_stick_y * 2,
                    autoAlign ? move.powerToAngle(alignAngle, 5) : gamepad1.right_stick_x,
                    power, 0.1);
        else // Traditional drive forward relative to robot
            robot.chassis.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, power);

        if(gamepad1.b && !trainingWheels) // Burnout
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

package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

public class OdometryBase implements Runnable {
    private DcMotor vlEncoder, vrEncoder, hEncoder;

    private Pose robotPose = new Pose(0,0,0);
    private final RobotHardware robot;

    private final double CPI = 141; //149.8366446 / 2;

    public static final String[] configNames =  Constants.chassis;

    private final int vlMultiplier = 1, vrMultiplier = 1, hMultiplier = 1,
            sleepTime = 5;  // Measured in milliseconds

    private final double mass = 12.5; // robot's mass in kg

    private double vlPosLast = 0, vrPosLast = 0, horPosLast = 0, lastHeading = 0,
                   oldXVelocity = 0, oldYVelocity = 0, oldWVelocity = 0,
                   vlPos, vrPos, hPos, vPos;
    public double xVelocity, yVelocity, wVelocity = 0,
                  xAcc = 0, yAcc = 0, wAcc = 0;

    private boolean isRunning = true;
    public boolean showPosition = true, showMovement = true, showAllData = false;

    private long time, previousTime = System.currentTimeMillis();

    public OdometryMovement move;

    /**
     * Provide the RobotHardware instance
     * @param robot Robot Hardware instance
     */
    public OdometryBase(RobotHardware robot) {
        this.robot = robot;
        move = new OdometryMovement(this, robot);
    }

    /**
     * @param robot The RobotHardware instance to use
     * @param startPose The pose to tell the system where the robot is on the field.
     */
    public OdometryBase(RobotHardware robot, Pose startPose) {
        this.robotPose = startPose;
        this.robot = robot;
        move = new OdometryMovement(this, robot);
    }

    /**
     * Overrides the position of the robot. This should be helpful for resetting odometry.
     * The angle will be reset to the imu angle in the next iteration of the tracking method.
     * @param newPose The new position and rotation to use.
     */
    public void overridePosition(Pose newPose){
        robotPose = newPose;
        robot.gyro.offset += newPose.getTheta();
    }

    /**
     * Initialize the encoders
     * @param hm Hardware map to use
     */
    public void init(HardwareMap hm){
        vlEncoder = hm.dcMotor.get(configNames[0]);
        vrEncoder = hm.dcMotor.get(configNames[2]);
        hEncoder = hm.dcMotor.get(configNames[1]);

        // Encoder wheel setup
        vlEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vrEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // It is RUN_WITHOUT_ENCODER because that is what allows us to run using the power values
        // We are still able to retrieve the encoder values accurately.
        vlEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vrEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // When the vertical wheels move forward they should be positive
        // When the horizontal wheels go to the right it should be positive
    }

    /**
     * Keeps track of the encoder positions and updates the robot position with some trig
     */
    private void trackPosition(){
        time = System.currentTimeMillis();

        vlPos = vlMultiplier * vlEncoder.getCurrentPosition() / CPI - vlPosLast;
        vrPos = vrMultiplier * vrEncoder.getCurrentPosition() / CPI - vrPosLast;
        hPos = hMultiplier * hEncoder.getCurrentPosition() / CPI - horPosLast;
        vPos = (vlPos + vrPos) / 2;

        double heading = robot.gyro.getHeading();

        // The horizontal encoder is 90 degrees from the other encoders so we need to use the co function of what the other encoders use
        double deltaX =  -getYComponent(heading, vPos) + getXComponent(heading, hPos);  // Forward (Y) change
        double deltaY = getXComponent(heading, vPos) + getYComponent(heading, hPos);  // Starboard (X) change

        robotPose.addX(deltaX);
        robotPose.addY(deltaY);
        robotPose.setTheta(heading);

        vlPosLast += vlPos;
        vrPosLast += vrPos;
        horPosLast += hPos;

        // Calculate Velocity
        double deltaT = (time - previousTime) / 1000d;

        xVelocity = deltaX / deltaT; // in/s
        yVelocity = deltaY / deltaT; // in/s
        wVelocity = (heading - lastHeading) / deltaT; // degrees/sec

        lastHeading = heading;

        // Calculate Acceleration
        xAcc = (xVelocity - oldXVelocity) / deltaT;
        yAcc = (yVelocity - oldYVelocity) / deltaT;
        wAcc = (wVelocity - oldWVelocity) / deltaT;
        robot.spinner.update(deltaT); // Update the duck spinner

        oldXVelocity = xVelocity;
        oldYVelocity = yVelocity;
        oldWVelocity = wVelocity;

        previousTime = time;

        // Print the XYTheta values to the telemetry.
        if (showPosition)
            robotPose.print(robot.gt);

        if (showMovement) {
            robot.gt.addData("Delta Time: ", deltaT * 1000 + "ms");

            // Print the robot's velocity to the telemetry

            robot.gt.addData("Velocity",
                    String.format("X: %.2fin/s | Y: %.2fin/s | W: %.2fdeg/s",
                            xVelocity, yVelocity, wVelocity));

            // Print the robot's Acceleration to the telemetry
            robot.gt.addData("Acc:",
                    String.format("X: %.2fin/s/s | Y: %.2fin/s/s | W: %.2fdeg/s/s",
                            xAcc, yAcc, wAcc));
        }

        // Print all the rest of the data to the telemetry.
        if (showAllData){
            robot.gt.addData("vlPos", vlPos);
            robot.gt.addData("vrPos", vrPos);
            robot.gt.addData("hPos", hPos);
            robot.gt.addData("vPos", vPos);
            robot.gt.addData("vlCounts", vlEncoder.getCurrentPosition() * vlMultiplier + "\nlast vlpos: " + (vlPosLast * CPI));
            robot.gt.addData("vrCounts", vrEncoder.getCurrentPosition() * vrMultiplier + "\nlast vrpos: " + (vrPosLast * CPI));
            robot.gt.addData("hCounts", hEncoder.getCurrentPosition() * hMultiplier + "\nlast hpos: " + (horPosLast * CPI));
            robot.gt.addData("Delta X", deltaX);
            robot.gt.addData("Delta Y", deltaY);
            robot.gt.addData("Heading", heading);
        }

        robot.gt.print();
    }

    /**
     * Prints the raw encoder values for the odometry wheels.
     */
    public void printEncoderPositions(){
        robot.gt.addData("vlPos", vlEncoder.getCurrentPosition());
        robot.gt.addData("vrPos", vrEncoder.getCurrentPosition());
        robot.gt.addData("hPos", hEncoder.getCurrentPosition());
        robot.gt.print();
    }

    /**
     * Runs the position tracking thread
     */
    @Override
    public void run() {
        while(isRunning) {
            trackPosition();
//            try {
//                Thread.sleep(sleepTime);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
        }
    }

    /**
     * Stops the position update thread
     */
    public void stop(){
        isRunning = false;
        robot.spinner.stop();
        robot.gyro.stop();
    }

    /**
     * Gets the forward (y) component of a vector with a certain angle in degrees and a magnitude
     * @param degrees The angle to use
     * @param magnitude The amount of distance traveled at that angle
     * @return The amount of distance traveled along the Y axis
     */
    private double getYComponent(double degrees, double magnitude){
        return Math.sin(Math.toRadians(degrees)) * magnitude;
    }

    /**
     * Gets the rightward (X) component of a vector with a certain angle in degrees and a magnitude
     * @param degrees The angle to use
     * @param magnitude The amount of distance traveled at that angle
     * @return The amount of distance traveled along the X axis
     */
    private double getXComponent(double degrees, double magnitude){
        return Math.cos(Math.toRadians(degrees)) * magnitude;
    }

    /**
     * Returns the robot pose in order to get the position of the robot
     * @return A pose object with the robot's X, Y, and Rotation
     */
    public Pose getRobotPose() {
        return robotPose;
    }
}

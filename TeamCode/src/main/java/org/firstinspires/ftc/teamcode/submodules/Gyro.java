package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Gyro implements Runnable {
    private BNO055IMU imu;

    public double offset = 0;
    double heading = 0;

    private boolean isRunning = true;

    void updateHeading(){
        heading = imu.getAngularOrientation().firstAngle - offset; // gets angle Z
    }

    /**
     * @return IMU heading in degrees where 0 is forward, 90 is to the left, -90 is to the right
     */
    public double getHeading(){
        return heading;
    }

    /**
     * Initialize the BNO55IMU
     * @param hardwareMap
     */
    public void init(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, Constants.gyro);
        imu.initialize(parameters);
    }

    @Override
    public void run() {
        while (isRunning){
            updateHeading();
        }
    }

    public void stop(){ isRunning = false; }

}

package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Gyro {
    private BNO055IMU imu;

    /**
     * Returns the heading of the robot where 0 is forward and 45 is to the right
     * @return IMU heading
     */
    public double getHeading(){
        return imu.getAngularOrientation().firstAngle; // gets angle Z
    }

    /**
     * Returns the angle of the robot where 0 is along the x-axis (right)
     * and 90 is along the y-axis (forward)
     * @return Degrees
     */
    public double getAngle(){
        return 90 - (imu.getAngularOrientation().firstAngle * 2);
    }

    /**
     * Returns the angle of the robot where 0 is along the x-axis (right)
     * and 90 is along the y-axis (forward)
     * @return Radians
     */
    public double getAngleRad(){
        return Math.toRadians(getAngle());
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
}

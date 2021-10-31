package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double mmPerInch = 25.4;

    public static final double robotwidth = 18;
    public static final double mmTargetHeight   = 6 * mmPerInch;

    public static final double fieldLength = 12 * 12;
    public static final double fieldWidth = 12 * 12;
    public static final double quadFieldLength = fieldLength / 4;
    public static final double halfFieldLength = fieldLength / 4;

    public static final double WHEEL_RADIUS = 2.5; // Measure the radius as precisely as possible
    public static final double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI;
    public static final double COUNTS_PER_REV = 28;
    public static final double COUNTS_PER_INCH = COUNTS_PER_REV / WHEEL_CIRCUMFERENCE; // replace with real value

    public static final double X_BOUND_LOW = 10, X_BOUND_HIGH = fieldLength - 10,
            Y_BOUND_LOW = 10, Y_BOUND_HIGH = fieldWidth - 10;

    // Configuration names
    public static final String webcam = "Webcam 1";
    public static final String gyro = "imu";
    public static final String[] chassis = {"front_left","front_right","back_right","back_left"};

}

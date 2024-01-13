package frc.robot.subsystems.drive;

import frc.robot.Constants;

public class DriveCal {
    public static final int SPARK_INIT_RETRY_ATTEMPTS = 5;

    /** Input meters/second, output [-1,1] */
    public static final double DRIVING_P = 0.1,
        DRIVING_I = 0.0,
        DRIVING_D = 0.1,
        DRIVING_FF = 0.95 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    /** Input radians, output [-1,1] */
    public static final double TURNING_P = 0.8,
        TURNING_I = Constants.PLACEHOLDER_DOUBLE,
        TURNING_D = 0.1,
        TURNING_FF = 0.00;

    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;
}

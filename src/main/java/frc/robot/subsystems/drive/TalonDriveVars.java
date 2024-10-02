package frc.robot.subsystems.drive;

import frc.robot.Constants;

public class TalonDriveVars {
  public static final double SENSOR_TO_MECHANISM_RATIO_DRIVE = Constants.PLACEHOLDER_DOUBLE;
  public static final double SENSOR_TO_MECHANISM_RATIO_TURNING = Constants.PLACEHOLDER_DOUBLE;
  public static final double DRIVING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;
  public static final double DRIVING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;
  public static final double TURNING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;
  public static final double TURNING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;

  public static final double DRIVING_P = 0.2,
      DRIVING_I = 0.0,
      DRIVING_D = 0.003,
      DRIVING_FF = 1.1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

  public static final double TURNING_P = 0.2,
      TURNING_I = 0.0,
      TURNING_D = 0.003,
      TURNING_FF = 1.1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

  public static final double TURNING_ENCODER_ZERO_THRESHOLD_ROTATION = Constants.PLACEHOLDER_DOUBLE;
}

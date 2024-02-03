package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
  /**
   * Invert the turning encoder, since the output shaft rotates in the opposite direction of the
   * steering motor in the MAXSwerve Module.
   */
  public static final boolean TURNING_ENCODER_INVERTED = true;

  /** Multiplier for wheel diameter based on empirical on-field measurement */
  public static final double WHEEL_DIAMETER_FUDGE_FACTOR = 1.0;

  /** Calculations required for driving motor conversion factors and feed forward */
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = Constants.NEO_FREE_SPEED_RPM / 60,
      WHEEL_DIAMETER_METERS = Units.inchesToMeters(3) * WHEEL_DIAMETER_FUDGE_FACTOR,
      WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

  public static final double DRIVING_MOTOR_MAX_ACCEL_RPS_SQRD = Constants.PLACEHOLDER_DOUBLE;

  /** the distance from the center of the robot to the furthest module */
  public static final double DRIVE_BASE_RADIUS_METERS = Constants.PLACEHOLDER_DOUBLE;

  public static final double DRIVING_MOTOR_REDUCTION = Constants.PLACEHOLDER_DOUBLE;
  public static final double DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR = 1.0;
  public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND =
      DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR
          * ((DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION);
  public static final double DRIVE_WHEEL_MAX_ACCEL_METERS_PER_SEC_SQRD =
      Constants.PLACEHOLDER_DOUBLE;

  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
      2 * Math.PI; // radians

  public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

  public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 50; // amps
  public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps

  public static final double TURN_MODULE_ENCODER_GEAR_RATIO = 1.0;

  public static final double MEDIUM_LINEAR_SPEED_METERS_PER_SEC = Constants.PLACEHOLDER_DOUBLE,
      MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = Constants.PLACEHOLDER_DOUBLE,
      MEDIUM_ANGULAR_SPEED_RAD_PER_SEC = Constants.PLACEHOLDER_DOUBLE,
      MEDIUM_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Constants.PLACEHOLDER_DOUBLE;
  public static final double SLOW_LINEAR_SPEED_METERS_PER_SEC = Constants.PLACEHOLDER_DOUBLE,
      SLOW_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = Constants.PLACEHOLDER_DOUBLE,
      SLOW_ANGULAR_SPEED_RAD_PER_SEC = Constants.PLACEHOLDER_DOUBLE,
      SLOW_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Constants.PLACEHOLDER_DOUBLE;

  /** Distance between centers of right and left wheels on robot */
  public static final double TRACK_WIDTH_METERS = Constants.PLACEHOLDER_DOUBLE;

  /**
   * Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather
   * the allowed maximum speeds
   */
  public static final double MAX_SPEED_METERS_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
      MAX_ANGULAR_SPEED_RAD_PER_SECONDS = Math.PI * 2;

  /** Distance between front and back wheels on robot */
  public static final double WHEEL_BASE_METERS = Constants.PLACEHOLDER_DOUBLE;

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE_METERS / 2, DriveConstants.TRACK_WIDTH_METERS / 2),
          new Translation2d(WHEEL_BASE_METERS / 2, -DriveConstants.TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, DriveConstants.TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, -DriveConstants.TRACK_WIDTH_METERS / 2));

  public static final boolean GYRO_REVERSED = false;
}

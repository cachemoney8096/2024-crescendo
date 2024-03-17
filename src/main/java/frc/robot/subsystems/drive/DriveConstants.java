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
  public static final double WHEEL_DIAMETER_FUDGE_FACTOR = 1.035 * 1.02 * 0.972;

  public static final double inchesToMeters = 0.0254;

  /** Calculations required for driving motor conversion factors and feed forward */
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = Constants.KRAKEN_FREE_SPEED_RPM / 60,
      WHEEL_DIAMETER_METERS = 3 * inchesToMeters * WHEEL_DIAMETER_FUDGE_FACTOR,
      WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

  /** the distance from the center of the robot to the furthest module */
  public static final double DRIVE_BASE_RADIUS_METERS = Units.inchesToMeters(15.2);

  public static final double DRIVING_MOTOR_REDUCTION = 3.75;
  public static final double DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR = 1.0;
  public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND =
      DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR
          * ((DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION);

  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
      2 * Math.PI; // radians

  public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

  public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS_AUTO = 50; // amps
  public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS_TELEOP = 40; // amps
  public static final int DRIVING_MOTOR_STATOR_CURRENT_LIMIT_AMPS = 120; // amps
  public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps

  public static final double TURN_MODULE_ENCODER_GEAR_RATIO = 1.0;

  /** Distance between centers of right and left wheels on robot */
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(21.5);

  /**
   * Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather
   * the allowed maximum speeds. Max speed is 85% free speed
   */
  public static final double FREE_SPEED_FUDGE_FACTOR = 0.95,
      MAX_SPEED_METERS_PER_SECOND = FREE_SPEED_FUDGE_FACTOR * Units.feetToMeters(20.25),
      MAX_ANGULAR_SPEED_RAD_PER_SECONDS = Math.PI * 2;

  /** Distance between front and back wheels on robot */
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(21.5);

  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE_METERS / 2, DriveConstants.TRACK_WIDTH_METERS / 2),
          new Translation2d(WHEEL_BASE_METERS / 2, -DriveConstants.TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, DriveConstants.TRACK_WIDTH_METERS / 2),
          new Translation2d(-WHEEL_BASE_METERS / 2, -DriveConstants.TRACK_WIDTH_METERS / 2));

  public static final boolean GYRO_REVERSED = false;

  /** time needed for the robot to turn 180 degrees when passed into drive.turnInPlace */
  public static final double TIME_TO_ROTATE_180_DEGREES = Constants.PLACEHOLDER_DOUBLE;
}

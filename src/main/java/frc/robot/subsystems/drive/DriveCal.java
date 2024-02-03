package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class DriveCal {
  public static final int SPARK_INIT_RETRY_ATTEMPTS = 5;

  /** Input meters/second, output [-1,1] */
  public static final double DRIVING_P = 0.1,
      DRIVING_I = 0.0,
      DRIVING_D = 0.1,
      DRIVING_FF = 0.95 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

  /** Input radians, output [-1,1] */
  public static final double TURNING_P = 0.8,
      TURNING_I = Constants.PLACEHOLDER_DOUBLE,
      TURNING_D = 0.1,
      TURNING_FF = 0.00;

  /**
   * Angular offset of the modules relative to the zeroing fixture in radians. Ideally should be
   * relative to the fixture but they are actually slightly different.
   */
  public static double SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE,
      SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE,
      SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE,
      SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE;

  /**
   * Angular offsets of the modules relative to the chassis in radians. The modules form an O when
   * fixtured, so they are iteratively 90 deg from each other.
   */
  public static final double
      FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD - (3.0 * Math.PI / 4.0),
      FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD - (Math.PI / 4.0),
      BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD + (3.0 * Math.PI / 4.0),
      BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD + (Math.PI / 4.0);

  /** Controller on module speed for rotating to target, input degrees [-180,180], output [0,1]. */
  public static final PIDController ROTATE_TO_TARGET_PID_CONTROLLER =
      new PIDController(0.015, 0, 0.000); // From 2022

  /** Feed forward for rotating to target, gets added to or subtracted from PID controller. */
  public static final double ROTATE_TO_TARGET_FF = 0.01;

  /** If the desired chassis rotation is below this value in [0,1], it is ignored */
  public static final double ROTATION_DEADBAND_THRESHOLD = 0.04;

  /** path finding controller for translation and rotation; used in PathPlanner */
  public static final PIDConstants
      PATH_TRANSLATION_CONTROLLER =
          new PIDConstants(
              9.0,
              0.0,
              0.0),
      PATH_ROTATION_CONTROLLER =
          new PIDConstants(
              9.0,
              0.0,
              0.0);
}

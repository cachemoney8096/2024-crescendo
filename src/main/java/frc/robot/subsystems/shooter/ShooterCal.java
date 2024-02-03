package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class ShooterCal {
  /** Motor A PID, Unit Inputs: RPM. output [-1,1] */
  public static final double MOTOR_A_kP = Constants.PLACEHOLDER_DOUBLE,
      MOTOR_A_kI = Constants.PLACEHOLDER_DOUBLE,
      MOTOR_A_kD = Constants.PLACEHOLDER_DOUBLE,
      MOTOR_A_kFF = Constants.PLACEHOLDER_DOUBLE;
  /** Motor B PID, Unit Inputs: RPM. output [-1,1] */
  public static final double MOTOR_B_kP = Constants.PLACEHOLDER_DOUBLE,
      MOTOR_B_kI = Constants.PLACEHOLDER_DOUBLE,
      MOTOR_B_kD = Constants.PLACEHOLDER_DOUBLE,
      MOTOR_B_kFF = Constants.PLACEHOLDER_DOUBLE;

  /** Pivot Motor PID, Unit Inputs: Degrees. output Volts */
  public static final double PIVOT_MOTOR_kP = Constants.PLACEHOLDER_DOUBLE,
      PIVOT_MOTOR_kI = Constants.PLACEHOLDER_DOUBLE,
      PIVOT_MOTOR_kD = Constants.PLACEHOLDER_DOUBLE;

  /**
   * Pivot Motor Feedforward. Inputs: Degrees. output Volts.
   */
  public static final double PIVOT_MOTOR_KS = Constants.PLACEHOLDER_DOUBLE,
      PIVOT_MOTOR_KV = Constants.PLACEHOLDER_DOUBLE,
      PIVOT_MOTOR_KA = Constants.PLACEHOLDER_DOUBLE;

  /**
   * Pivot Motor Feedforward. Inputs: Degrees. output Volts.
   */
  public static final SimpleMotorFeedforward PIVOT_MOTOR_FF =
      new SimpleMotorFeedforward(PIVOT_MOTOR_KS, PIVOT_MOTOR_KV, PIVOT_MOTOR_KA);

  /** Goal position at the start of a match. */
  public static final double STARTING_POSITION_DEGREES = Constants.PLACEHOLDER_DOUBLE;

  /** Shooter is spun up if within this margin of the goal speed (rev/min) */
  public static final double SHOOTER_SPEED_MARGIN_RPM = 40.0;

  /** Motion profile constraints for the pivot. */
  public static final double PIVOT_MAX_VELOCITY_DEG_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
      PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = Constants.PLACEHOLDER_DOUBLE;

  /** Pivot motor voltage to hold the pivot in place at  */
  public static final double ARBITRARY_PIVOT_FEED_FORWARD_VOLTS = Constants.PLACEHOLDER_DOUBLE;
  public static final int SHOOTER_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_INT;

  /** Pivot position for latching onto the chain */
  public static final double LATCH_ANGLE_DEGREES = Constants.PLACEHOLDER_DOUBLE;
  /** Pivot position for climbing, to be able to latch the chain later */
  public static final double PRE_LATCH_ANGLE_DEGREES = Constants.PLACEHOLDER_DOUBLE;

  /** How close to goal position to be considered "there" */
  public static final double PIVOT_ANGLE_MARGIN_DEG = Constants.PLACEHOLDER_DOUBLE;

  /** Target shooter motor speeds (at the motor) */
  public static final double SHOOTER_MOTOR_SPEED_RPM = Constants.PLACEHOLDER_DOUBLE;
}

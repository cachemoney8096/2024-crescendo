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
  public static final double PIVOT_MOTOR_kP = 0.2,
      PIVOT_MOTOR_kI = Constants.PLACEHOLDER_DOUBLE,
      PIVOT_MOTOR_kD = 0.01;


  /**
   * Pivot Motor Feedforward. Inputs: Degrees. output Volts.
   * https://www.reca.lc/arm?armMass=%7B%22s%22%3A20%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=85&endAngle=%7B%22s%22%3A30%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A127.5%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
   */
  public static final double PIVOT_MOTOR_KS = 0.1,
      PIVOT_MOTOR_KV = 0.043,
      PIVOT_MOTOR_KA = 0.000083;

  /** Pivot Motor Feedforward. Inputs: Degrees. output Volts. */
  public static final SimpleMotorFeedforward PIVOT_MOTOR_FF =
      new SimpleMotorFeedforward(PIVOT_MOTOR_KS, PIVOT_MOTOR_KV, PIVOT_MOTOR_KA);

  /** Goal position at the start of a match. */
  public static final double STARTING_POSITION_DEGREES = 80.0;

  /** Shooter is spun up if within this margin of the goal speed (rev/min) */
  public static final double SHOOTER_SPEED_MARGIN_RPM = 40.0;

  /** Motion profile constraints for the pivot. */
  public static final double PIVOT_MAX_VELOCITY_DEG_PER_SECOND = 200.0,
      PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = 480.0;

  public static final double PIVOT_PROFILE_REPLANNING_THRESHOLD = 5.0;
  public static final double PIVOT_ENCODER_ZEROING_THRESHOLD = 5.0;

  /** Pivot motor voltage to hold the pivot in place at */
  public static final double PIVOT_GRAVITY_FEED_FORWARD_VOLTS = 0.28;

  /** Pivot motor arbitrary voltage in direction of position error */
  public static final double PIVOT_POS_FEED_FORWARD_VOLTS = 0.1;

  public static final int SHOOTER_CURRENT_LIMIT_AMPS = 50;
  public static final int PIVOT_CURRENT_LIMIT_AMPS = 50;

  /** Pivot position for latching onto the chain */
  public static final double LATCH_ANGLE_DEGREES = 110.0;
  /** Pivot position for climbing, to be able to latch the chain later */
  public static final double PRE_LATCH_ANGLE_DEGREES = 80.0;

  /** How close to goal position to be considered "there" */
  public static final double PIVOT_ANGLE_MARGIN_DEG = 0.5;

  /** Target shooter motor speeds (at the motor) */
  public static final double SHOOTER_MOTOR_SPEED_RPM = 4000.0;

  /**
   * What the abs encoder reads (in degrees) when the shooter is pointed down. Acquired by pointing
   * the encoder at 90 and subtracting 90 from that reading.
   */
  public static final double PIVOT_ANGLE_OFFSET_DEGREES = 39.5 - 90.0;

  /** Position at which (or beyond) the shooter has potential to intersect the conveyor. */
  public static final double CONVEYOR_ZONE_THRESHOLD_DEGREES = Constants.PLACEHOLDER_DOUBLE;
}

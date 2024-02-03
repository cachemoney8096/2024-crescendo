package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public final class ElevatorCal {
  /** Feedback parameters for elevator when not climbing. Input in/s, output Volts */
  public static final double NOTE_SCORING_P = Constants.PLACEHOLDER_DOUBLE,
      NOTE_SCORING_I = Constants.PLACEHOLDER_DOUBLE,
      NOTE_SCORING_D = Constants.PLACEHOLDER_DOUBLE;
  /** Feedforward parameters for the elevator when not climbing. Input in/s, output Volts. */
  public static final double NOTE_SCORING_KS = Constants.PLACEHOLDER_DOUBLE,
      NOTE_SCORING_KV = Constants.PLACEHOLDER_DOUBLE,
      NOTE_SCORING_KA = Constants.PLACEHOLDER_DOUBLE;

  public static final SimpleMotorFeedforward NOTE_SCORING_FF =
      new SimpleMotorFeedforward(NOTE_SCORING_KS, NOTE_SCORING_KV, NOTE_SCORING_KA);

  /** Feedback parameters for elevator when climbing. Input in/s, output Volts. */
  public static final double CLIMBING_P = Constants.PLACEHOLDER_DOUBLE,
      CLIMBING_I = Constants.PLACEHOLDER_DOUBLE,
      CLIMBING_D = Constants.PLACEHOLDER_DOUBLE;
  /** Feedforward parameters for the elevator when climbing. Input in/s, output Volts. */
  public static final double CLIMBING_KS = Constants.PLACEHOLDER_DOUBLE,
      CLIMBING_KV = Constants.PLACEHOLDER_DOUBLE,
      CLIMBING_KA = Constants.PLACEHOLDER_DOUBLE;

  public static final SimpleMotorFeedforward CLIMBING_FF =
      new SimpleMotorFeedforward(CLIMBING_KS, CLIMBING_KV, CLIMBING_KA);

  /** Elevator trapezoidal motion profile constraints. */
  public static final double MAX_VELOCITY_IN_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED = Constants.PLACEHOLDER_DOUBLE;

  /** Elevator positions relative to home position */
  public static final double POSITION_HOME_INCHES = Constants.PLACEHOLDER_DOUBLE,
      POSITION_SCORE_TRAP_INCHES = Constants.PLACEHOLDER_DOUBLE,
      POSITION_SCORE_AMP_INCHES = Constants.PLACEHOLDER_DOUBLE,
      POSITION_PRE_CLIMB_INCHES = Constants.PLACEHOLDER_DOUBLE,
      POSITION_POST_CLIMB_INCHES = Constants.PLACEHOLDER_DOUBLE;

  /** Soft limits for motors relative to home position. */
  public static final float ELEVATOR_POSITIVE_LIMIT_INCHES = Constants.PLACEHOLDER_FLOAT,
      ELEVATOR_NEGATIVE_LIMIT_INCHES = Constants.PLACEHOLDER_FLOAT;

  public static final int ELEVATOR_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_INT;

  /** Consider the elevator at the desired position if within this margin of the target. */
  public static final double ELEVATOR_MARGIN_INCHES = 0.5;

  /**
   * if elevator is below the threshold, the intake is safe to move without first moving the
   * elevator this value should be the bottom of the range of values that cause intake-elevator
   * interference
   */
  public static final double ELEVATOR_INTERFERENCE_THRESHOLD_INCHES = Constants.PLACEHOLDER_DOUBLE;
}

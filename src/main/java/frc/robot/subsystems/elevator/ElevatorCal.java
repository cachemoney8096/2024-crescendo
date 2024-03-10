package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public final class ElevatorCal {
  /** Feedback parameters for elevator when not climbing. Input in, output Volts */
  public static final double NOTE_SCORING_P = 1.0, NOTE_SCORING_I = 0, NOTE_SCORING_D = 0;
  /**
   * Feedforward parameters for the elevator when not climbing. Input in/s, output Volts.
   * https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=85&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A19.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A7.92%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A17%2C%22u%22%3A%22in%22%7D
   */
  public static final double NOTE_SCORING_KS = 0.0,
      NOTE_SCORING_KV = 0.26,
      NOTE_SCORING_KA = 0.00076;

  public static final SimpleMotorFeedforward NOTE_SCORING_FF =
      new SimpleMotorFeedforward(0.0, NOTE_SCORING_KV, NOTE_SCORING_KA);

  /** Feedback parameters for elevator when climbing. Input in/s, output Volts. */
  public static final double CLIMBING_P = 1.0, CLIMBING_I = 0, CLIMBING_D = 0;
  /**
   * Feedforward parameters for the elevator when climbing. Input in/s, output Volts.
   * https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A70%2C%22u%22%3A%22A%22%7D&efficiency=85&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A110%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A7.92%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A17%2C%22u%22%3A%22in%22%7D
   */
  public static final double CLIMBING_KS = -2.5, CLIMBING_KV = 0.26, CLIMBING_KA = 0.0043;

  public static final SimpleMotorFeedforward CLIMBING_FF =
      new SimpleMotorFeedforward(0.0, CLIMBING_KV, CLIMBING_KA);

  /** Elevator trapezoidal motion profile constraints. */
  public static final double MAX_VELOCITY_IN_PER_SECOND = 40.0,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED = 640.0;
  /** Elevator trapezoidal motion profile constraints. */
  public static final double MAX_VELOCITY_IN_PER_SECOND_CLIMB = 40.0,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED_CLIMB = 40.0;

  /** Elevator positions relative to home position. Home position fixed at 3 inches. */
  public static final double POSITION_HOME_INCHES = 2.75,
      POSITION_SCORE_TRAP_INCHES = 38.5,
      POSITION_SCORE_AMP_INCHES = 32.5,
      POSITION_PRE_CLIMB_INCHES = 38.0,
      POSITION_SLIGHTLY_UP_INCHES = 3.75,
      POSITION_INTAKE_INCHES = 5.75;

  public static final int ELEVATOR_CURRENT_LIMIT_AMPS = 80;

  /** Consider the elevator at the desired position if within this margin of the target. */
  public static final double ELEVATOR_MARGIN_INCHES = 0.5;

  /**
   * if elevator is below the bottom of the zone or above the top of the zone, the intake is safe to
   * move without first moving the elevator. This value should be the bottom or top, respectively,
   * of the range of values that cause intake-elevator interference
   */
  public static final double ELEVATOR_INTERFERENCE_THRESHOLD_MINIMUM_INCHES = 4.25,
      ELEVATOR_INTERFERENCE_THRESHOLD_MAXIMUM_INCHES = 18.5;
}

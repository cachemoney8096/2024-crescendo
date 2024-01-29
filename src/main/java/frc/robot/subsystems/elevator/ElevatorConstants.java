package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public final class ElevatorConstants {
  public static final int MAX_INIT_RETRY_ATTEMPTS = 5;

  public static final double ELEVATOR_DRUM_DIAMETER_IN = Constants.PLACEHOLDER_DOUBLE;

  /** Elevator motor to drum. >1 meaning reduction. */
  public static final double ELEVATOR_GEAR_RATIO = Constants.PLACEHOLDER_DOUBLE;

  /** Ratio from the left encoder to the drum. >1 meaning the encoder spins slower. */
  public static final double ELEVATOR_LEFT_ABSOLUTE_ENCODER_RATIO = Constants.PLACEHOLDER_DOUBLE;

  /** Ratio from the right encoder to the drum. >1 meaning the encoder spins slower. */
  public static final double ELEVATOR_RIGHT_ABSOLUTE_ENCODER_RATIO = Constants.PLACEHOLDER_DOUBLE;
}
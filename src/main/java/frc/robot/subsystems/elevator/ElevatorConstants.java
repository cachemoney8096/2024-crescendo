package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
  public static final int MAX_INIT_RETRY_ATTEMPTS = 5;

  public static final double ELEVATOR_DRUM_DIAMETER_FUDGE_FACTOR = 1.03;

  /**
   * Fudge factor based on empirical measurements. Value multiplied based on actual drum diameter
   * plus one diameter of string.
   */
  public static final double ELEVATOR_DRUM_DIAMETER_IN = ELEVATOR_DRUM_DIAMETER_FUDGE_FACTOR * 1.07;

  /** Elevator motor to drum. >1 meaning reduction. */
  public static final double ELEVATOR_GEAR_RATIO = 7.92;

  /** Ratio from the left encoder to the drum. >1 meaning the encoder spins slower. */
  public static final double ELEVATOR_LEFT_ABSOLUTE_ENCODER_RATIO = 1.0;

  /** Ratio from the right encoder to the drum. >1 meaning the encoder spins slower. */
  public static final double ELEVATOR_RIGHT_ABSOLUTE_ENCODER_RATIO = 44.0 / 26.0;
}

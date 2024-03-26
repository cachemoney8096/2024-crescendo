package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IntakeCal {

  /* input degrees, output volts
   * https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A14%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=85&endAngle=%7B%22s%22%3A50%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A97%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
   */
  public static final SimpleMotorFeedforward INTAKE_PIVOT_FEEDFORWARD =
      new SimpleMotorFeedforward(0.4, 0.033, 0.00070);

  /** Voltage needed to hold the pivot up when it's horizontal */
  public static final double ARBITRARY_INTAKE_PIVOT_FEEDFORWARD_VOLTS = 1.0;

  public static final double INTAKE_HOLD_DEPLOYED_VOLTS = 0.75;

  /**
   * Offset degrees for intake absolute encoder. Real world degrees minus what the encoder read at
   * that position.
   */
  public static final double INTAKE_ABSOLUTE_ENCODER_ZERO_OFFSET_DEG = 195 - 106.4;

  public static final double PIVOT_ENCODER_ZEROING_THRESHOLD_DEG = 5.0;
  public static final double PIVOT_PROFILE_REPLANNING_THRESHOLD_DEG = 20.0;

  /**
   * The absolute encoder wraps every 120 degrees. At most stowed, it will read 5-10 degrees higher
   * than at most deployed (actually 110-115 deg lower). For positions at most deployed, the real
   * value is 120 deg greater due to wrap. This value is between the readings at the max travel in
   * each direction. If the reading is above vs. below, this indicates whether 120 deg needs to be
   * added to the reading.
   */
  public static final double INTAKE_ABSOLUTE_ENCODER_WRAP_POINT_DEG = 64.0;

  /** PID values for the intake pivot motor input degrees, output volts */
  public static final double INTAKE_PIVOT_P = 0.2, INTAKE_PIVOT_I = 0, INTAKE_PIVOT_D = 0;

  /** Motion profile max velocity and acceleration (deg per second) for intake pivot motor */
  public static final double PIVOT_MAX_VELOCITY_DEG_PER_SECOND = 330.0,
      PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = 950.0;

  /**
   * intake positions (degrees) where 90 degrees represents the intake at the shooter and 0 degrees
   * is down
   */
  public static final double INTAKE_DEPLOYED_POSITION_DEGREES = 270.4,
      INTAKE_STOWED_POSITION_DEGREES = 158.0,
      INTAKE_CLEAR_OF_CONVEYOR_DEGREES = 200,
      INTAKE_ALMOST_CLEAR_OF_CONVEYOR_DEGREES = 180,
      INTAKE_SAFE_POSITION_DEGREES = 200.0; // exact safe position is 186.0

  public static final double INTAKE_MARGIN_DEGREES = 8.0;
  /** Exact safe position is 186, so we use a higher threshold here to be safe */
  public static final double CONVEYOR_ZONE_THRESHOLD_DEGREES = 196.0;

/** Lower than this up to almost 270, intake blocks camera view */
  public static final double LIMELIGHT_BLOCKED_THRESHOLD = 202.0;
  

  /** intake power [-1.0,1.0] */
  public static final double INTAKING_POWER = 0.75, REVERSE_INTAKING_POWER = -1.0;

  public static final int PIVOT_MOTOR_CURRENT_LIMIT_AMPS = 50;
  public static final double INTAKING_TALONS_CURRENT_LIMIT_AMPS = 50.0;

  public static final double TALON_DUTY_CYCLE_UPDATE_FREQ_HZ = 50.0;
}

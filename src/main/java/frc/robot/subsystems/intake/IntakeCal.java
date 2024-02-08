package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class IntakeCal {

  /** PID values for the intake pivot motor input degrees, output volts */
  public static final double INTAKE_PIVOT_P = Constants.PLACEHOLDER_DOUBLE,
      INTAKE_PIVOT_I = Constants.PLACEHOLDER_DOUBLE,
      INTAKE_PIVOT_D = Constants.PLACEHOLDER_DOUBLE;

  /** Motion profile max velocity and acceleration (deg per second) for intake pivot motor */
  public static final double PIVOT_MAX_VELOCITY_DEG_PER_SECOND = 30.0,
      PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = 30.0;

  /**
   * intake positions (degrees) where 90 degrees represents the intake at the shooter and 0 degrees
   * is down
   */
  public static final double INTAKE_DEPLOYED_POSITION_DEGREES = 270.0,
      INTAKE_STOWED_POSITION_DEGREES = 155.0,
      INTAKE_SAFE_POSITION_DEGREES = 200.0; // exact safe position is 186.0

  public static final double INTAKE_MARGIN_DEGREES = 1.0,
      CONVEYOR_ZONE_THRESHOLD_DEGREES = 190.0;

  /** intake power [-1.0,1.0] */
  public static final double INTAKING_POWER = 1.0, REVERSE_INTAKING_POWER = -1.0;

  /* input degrees, output volts 
   * https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A14%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=85&endAngle=%7B%22s%22%3A50%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A97%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
  */
  public static final SimpleMotorFeedforward INTAKE_PIVOT_FEEDFORWARD =
      new SimpleMotorFeedforward(0.0, 0.033, 0.00070);

  /** Voltage needed to hold the pivot up when it's horizontal */
  public static final double ARBITRARY_INTAKE_PIVOT_FEEDFORWARD_VOLTS =
      0.76;

  /** Offset degrees for intake absolute encoder */
  public static final double INTAKE_ABSOLUTE_ENCODER_ZERO_OFFSET_DEG = Constants.PLACEHOLDER_DOUBLE;

  public static final int PIVOT_MOTOR_CURRENT_LIMIT_AMPS = 25;
  public static final double INTAKING_TALONS_CURRENT_LIMIT_AMPS = 50.0;
}

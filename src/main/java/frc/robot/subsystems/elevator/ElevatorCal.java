package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public final class ElevatorCal {
  public static final double NOTE_SCORING_P = Constants.PLACEHOLDER_DOUBLE,
  NOTE_SCORING_I = Constants.PLACEHOLDER_DOUBLE,
  NOTE_SCORING_D = Constants.PLACEHOLDER_DOUBLE;
  public static final SimpleMotorFeedforward NOTE_SCORING_FF = new SimpleMotorFeedforward(
            Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

  public static final double GRABBING_P = Constants.PLACEHOLDER_DOUBLE,
  GRABBING_I = Constants.PLACEHOLDER_DOUBLE,
  GRABBING_D = Constants.PLACEHOLDER_DOUBLE;
  public static final SimpleMotorFeedforward GRABBING_FF = new SimpleMotorFeedforward(
            Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

  public static final double MAX_VELOCITY_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
  MAX_ACCELERATION_PER_SECOND_SQUARED = Constants.PLACEHOLDER_DOUBLE;

  public static final double POSITION_HOME = Constants.PLACEHOLDER_DOUBLE,
  POSITION_SCORE_TRAP = Constants.PLACEHOLDER_DOUBLE,
  POSITION_SCORE_AMP = Constants.PLACEHOLDER_DOUBLE,
  POSITION_PRE_CLIMB = Constants.PLACEHOLDER_DOUBLE,
  POSITION_CLIMB = Constants.PLACEHOLDER_DOUBLE,
  POSITION_POST_CLIMB = Constants.PLACEHOLDER_DOUBLE;
  
  public static final int MAX_INIT_RETRY_ATTEMPTS = Constants.PLACEHOLDER_INT;

  public static final double ELEVATOR_RIGHT_ABSOLUTE_ENCODER_RATIO = Constants.PLACEHOLDER_DOUBLE;

  public static final double ELEVATOR_MOTOR_ENCODER_IN_PER_REV = Constants.PLACEHOLDER_DOUBLE, 
  ELEVATOR_MOTOR_ENCODER_IPS_PER_RPM = Constants.PLACEHOLDER_DOUBLE;

  public static final float ELEVATOR_POSITIVE_LIMIT_INCHES = Constants.PLACEHOLDER_FLOAT,
  ELEVATOR_NEGATIVE_LIMIT_INCHES = Constants.PLACEHOLDER_FLOAT;

  public static final int ELEVATOR_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_INT;
}

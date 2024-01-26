package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public final class ElevatorCal {
  public static final double NOTE_SCORING_P = Constants.PLACEHOLDER_DOUBLE,
  NOTE_SCORING_I = Constants.PLACEHOLDER_DOUBLE,
  NOTE_SCORING_D = Constants.PLACEHOLDER_DOUBLE,
  NOTE_SCORING_KS = Constants.PLACEHOLDER_DOUBLE,
  NOTE_SCORING_KV = Constants.PLACEHOLDER_DOUBLE,
  NOTE_SCORING_KA = Constants.PLACEHOLDER_DOUBLE;
  public static final SimpleMotorFeedforward NOTE_SCORING_FF = new SimpleMotorFeedforward(
            NOTE_SCORING_KS, NOTE_SCORING_KV, NOTE_SCORING_KA);

  public static final double CLIMBING_P = Constants.PLACEHOLDER_DOUBLE,
  CLIMBING_I = Constants.PLACEHOLDER_DOUBLE,
  CLIMBING_D = Constants.PLACEHOLDER_DOUBLE,
  CLIMBING_KS = Constants.PLACEHOLDER_DOUBLE,
  CLIMBING_KV = Constants.PLACEHOLDER_DOUBLE,
  CLIMBING_KA = Constants.PLACEHOLDER_DOUBLE;
  public static final SimpleMotorFeedforward CLIMBING_FF = new SimpleMotorFeedforward(
            CLIMBING_KS, CLIMBING_KV, CLIMBING_KA);

  public static final double MAX_VELOCITY_IN_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
  MAX_ACCELERATION_IN_PER_SECOND_SQUARED = Constants.PLACEHOLDER_DOUBLE;

  public static final double POSITION_IN_HOME = Constants.PLACEHOLDER_DOUBLE,
  POSITION_IN_SCORE_TRAP = Constants.PLACEHOLDER_DOUBLE,
  POSITION_IN_SCORE_AMP = Constants.PLACEHOLDER_DOUBLE,
  POSITION_IN_PRE_CLIMB = Constants.PLACEHOLDER_DOUBLE,
  POSITION_IN_POST_CLIMB = Constants.PLACEHOLDER_DOUBLE;
  
  public static final int MAX_INIT_RETRY_ATTEMPTS = 5;

  public static final double ELEVATOR_LEFT_ABSOLUTE_ENCODER_RATIO = Constants.PLACEHOLDER_DOUBLE,
   ELEVATOR_RIGHT_ABSOLUTE_ENCODER_RATIO = Constants.PLACEHOLDER_DOUBLE;

  public static final double ELEVATOR_MOTOR_ENCODER_IN_PER_REV = Constants.PLACEHOLDER_DOUBLE, 
  ELEVATOR_MOTOR_ENCODER_IN_PER_SEC_PER_RPM = Constants.PLACEHOLDER_DOUBLE;

  public static final float ELEVATOR_POSITIVE_LIMIT_INCHES = Constants.PLACEHOLDER_FLOAT,
  ELEVATOR_NEGATIVE_LIMIT_INCHES = Constants.PLACEHOLDER_FLOAT;

  public static final int ELEVATOR_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_INT;

  public static final double ELEVATOR_MARGIN_IN = Constants.PLACEHOLDER_DOUBLE;
}

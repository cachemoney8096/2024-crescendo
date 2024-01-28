package frc.robot.subsystems.shooter;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class ShooterCal {
    /** Motor A & B & pivotMotor, Unit Inputs: RPM. output [-1,1] */
  public static final double MOTOR_A_kP = Constants.PLACEHOLDER_DOUBLE,
    MOTOR_A_kI = Constants.PLACEHOLDER_DOUBLE,
    MOTOR_A_kD = Constants.PLACEHOLDER_DOUBLE,
    MOTOR_A_kFF = Constants.PLACEHOLDER_DOUBLE;

  public static final double MOTOR_B_kP = Constants.PLACEHOLDER_DOUBLE, 
    MOTOR_B_kI = Constants.PLACEHOLDER_DOUBLE, 
    MOTOR_B_kD = Constants.PLACEHOLDER_DOUBLE,
    MOTOR_B_kFF = Constants.PLACEHOLDER_DOUBLE;

  public static final double PIVOT_MOTOR_kP = Constants.PLACEHOLDER_DOUBLE,
    PIVOT_MOTOR_kI = Constants.PLACEHOLDER_DOUBLE,
    PIVOT_MOTOR_kD = Constants.PLACEHOLDER_DOUBLE;
    
  public static final double PIVOT_MOTOR_KS = Constants.PLACEHOLDER_DOUBLE,
    PIVOT_MOTOR_KV = Constants.PLACEHOLDER_DOUBLE,
    PIVOT_MOTOR_KA = Constants.PLACEHOLDER_DOUBLE;
  public static final SimpleMotorFeedforward PIVOT_MOTOR_FF = new SimpleMotorFeedforward(
            PIVOT_MOTOR_KS, PIVOT_MOTOR_KV, PIVOT_MOTOR_KA);

  public static final double ARBITRARY_FEED_FORWARD_VOLTS = Constants.PLACEHOLDER_DOUBLE;
  public static final double STARTING_POSITION_DEGREES = Constants.PLACEHOLDER_DOUBLE;

  public static final double PIVOT_MAX_VELOCITY_DEG_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
  PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = Constants.PLACEHOLDER_DOUBLE;

  public static final double ARBITRARY_PIVOT_FEED_FORWARD_VOLTS = Constants.PLACEHOLDER_DOUBLE;
  public static final int SHOOTER_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_INT;
}


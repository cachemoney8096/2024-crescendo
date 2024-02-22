package frc.robot;

public class RobotMap {
  public static final int PIGEON_CAN_ID = 20;
  /** Driving TalonFX CAN IDs */
  public static final int FRONT_LEFT_DRIVING_CAN_ID = 11,
      REAR_LEFT_DRIVING_CAN_ID = 16,
      FRONT_RIGHT_DRIVING_CAN_ID = 6,
      REAR_RIGHT_DRIVING_CAN_ID = 2;

  /** shooter SPARK MAX CAN IDS* */
  public static final int SHOOTER_MOTOR_RIGHT_CAN_ID = 4;

  public static final int SHOOTER_MOTOR_LEFT_ONE_CAN_ID = 13;
  public static final int SHOOTER_MOTOR_LEFT_TWO_CAN_ID = 14;
  public static final int SHOOTER_PIVOT_MOTOR_CAN_ID = 9;

  /** Turning SPARK MAX CAN IDs */
  public static final int FRONT_LEFT_TURNING_CAN_ID = 12,
      REAR_LEFT_TURNING_CAN_ID = 15,
      FRONT_RIGHT_TURNING_CAN_ID = 5,
      REAR_RIGHT_TURNING_CAN_ID = 3;

  /** Conveyor SPARK MAX CAN IDs */
  public static final int FRONT_CONVEYOR_CAN_ID = 18, BACK_CONVEYOR_CAN_ID = 19;

  public static final int LEFT_ELEVATOR_CAN_ID = 10, RIGHT_ELEVATOR_CAN_ID = 7;

  /** Intake motor and talon CAN IDs */
  public static final int INTAKE_PIVOT_MOTOR_CAN_ID = 8,
      INTAKING_LEFT_MOTOR_CAN_ID = 22,
      INTAKING_RIGHT_MOTOR_CAN_ID = 21;

  public static final int CANDLE_CAN_ID = Constants.PLACEHOLDER_INT;

  public static final int INTAKE_BEAM_BREAK_SENSOR_DIO = 9;
}

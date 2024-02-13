package frc.robot.subsystems.conveyor;

public class ConveyorCal {
  public static final int SPARK_INIT_RETRY_ATTEMPTS = 5;

  public static final int CONVEYOR_CURRENT_LIMIT_AMPS = 20;

  /** The power used for each conveyor motor when moving towards the shooter (forwards) */
  public static final double PREPARE_TO_SHOOT_FRONT_SPEED = 1.0, PREPARE_TO_SHOOT_BACK_SPEED = 1.0;

  /** The power used for each conveyor motor when moving towards the intake (backwards) */
  public static final double SCORE_AMP_TRAP_FRONT_SPEED = -1.0, SCORE_AMP_TRAP_BACK_SPEED = -1.0;

  /** The power used for each conveyor motor to receive the note */
  public static final double FRONT_RECEIVE_SPEED = 0.3;

  /**
   * The threshold required for the position of the back motor to change in order to register the
   * note
   */
  public static final double NOTE_POSITION_THRESHOLD_INCHES = 0.5;

  /** The time required for a note to exit the conveyor */
  public static final double NOTE_EXIT_TIME_SHOOTER_SECONDS = 0.5,
      NOTE_EXIT_TIME_TRAP_AMP_SECONDS = 0.5;

  /** The position to back off when receiving a note */
  public static final double BACK_OFF_INCHES = 1.0;

  /** The power to use when backing off a note */
  public static final double BACK_OFF_POWER = -1.0;
}

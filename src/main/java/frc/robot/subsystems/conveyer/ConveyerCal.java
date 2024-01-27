package frc.robot.subsystems.conveyer;

public class ConveyerCal {
  public static final int SPARK_INIT_RETRY_ATTEMPTS = 5;

  public static final int CONVEYER_CURRENT_LIMIT_AMPS = 20;

  /** The power used for each conveyer motor when moving towards the shooter (forwards) */
  public static final double PREPARE_TO_SHOOT_FRONT_SPEED = 1.0, PREPARE_TO_SHOOT_BACK_SPEED = 1.0;

  /** The power used for each conveyer motor when moving towards the intake (backwards) */
  public static final double SCORE_AMP_TRAP_FRONT_SPEED = -1.0,
      SCORE_AMP_TRAP_BACK_SPEED = -1.0;

  /** The power used for each conveyer motor to hold the note in place */
  public static final double FRONT_HOLD_SPEED = 1.0, BACK_HOLD_SPEED = -1.0;

  /** The threshold required for the position of the back motor to change in order to register the note */
  public static final double NOTE_THRESHOLD_DEGREES = 0.5;
}

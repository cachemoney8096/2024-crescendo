package frc.robot.subsystems.conveyer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;

public class Conveyer extends SubsystemBase {
  private CANSparkMax frontMotor =
      new CANSparkMax(RobotMap.FRONT_CONVEYER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax backMotor =
      new CANSparkMax(RobotMap.BACK_CONVEYER_CAN_ID, MotorType.kBrushless);

  private RelativeEncoder frontMotorEncoder = frontMotor.getEncoder();
  private RelativeEncoder backMotorEncoder = backMotor.getEncoder();

  /**
   * These aren't actually limit switches; we just use SparkLimitSwitch objects to access them
   * easily. At the moment, these are unused.
   */
  SparkLimitSwitch
      beamBreakSensorOne = frontMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen),
      beamBreakSensorTwo = frontMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen),
      beamBreakSensorThree = backMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen),
      beamBreakSensorFour = backMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  /**
   * Used to define the current position of the note.
   *
   * <ul>
   *   <li>NO_NOTE: There is no note in the conveyer.
   *   <li>HOLDING_NOTE: The conveyer is holding a note in place.
   *   <li>PARTIAL_NOTE: The conveyer is in the process of receiving a note from the intake, sending
   *       a note to the shooter, or scoring a note in the trap or amp.
   * </ul>
   */
  public enum ConveyerPositions {
    NO_NOTE,
    HOLDING_NOTE,
    PARTIAL_NOTE
  }

  /**
   * Used to define the current action of the conveyer.
   *
   * <ul>
   *   <li>NO_ACTION: The conveyer is not doing anything (i.e. either holding a note in place, or
   *       does not have a note at all)
   *   <li>SHOOT: The conveyer is sending a note towards the shooter.
   *   <li>TRAP_AMP: The conveyer is scoring in the trap or amp.
   *   <li>RECEIVE: The conveyer is receiving a note from the intake.
   * </ul>
   */
  public enum ConveyerActions {
    NO_ACTION,
    SHOOT,
    TRAP_AMP,
    RECEIVE,
  }

  public ConveyerPositions currentNotePosition = ConveyerPositions.NO_NOTE;
  public ConveyerPositions desiredNotePosition = ConveyerPositions.NO_NOTE;

  public ConveyerActions currentAction = ConveyerActions.NO_ACTION;
  public ConveyerActions desiredAction = ConveyerActions.NO_ACTION;

  public double secondMotorZeroPosition;

  public Conveyer() {
    SparkMaxUtils.initWithRetry(this::setUpConveyerSparks, ConveyerCal.SPARK_INIT_RETRY_ATTEMPTS);

    secondMotorZeroPosition = backMotorEncoder.getPosition();
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean setUpConveyerSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(frontMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(backMotor.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(frontMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(backMotor.setIdleMode(IdleMode.kBrake));

    frontMotor.setInverted(ConveyerConstants.FRONT_MOTOR_INVERTED);
    backMotor.setInverted(ConveyerConstants.BACK_MOTOR_INVERTED);

    errors +=
        SparkMaxUtils.check(
            frontMotor.setSmartCurrentLimit(ConveyerCal.CONVEYER_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            backMotor.setSmartCurrentLimit(ConveyerCal.CONVEYER_CURRENT_LIMIT_AMPS));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(frontMotorEncoder, 1.0));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(backMotorEncoder, 1.0));

    errors += SparkMaxUtils.check(beamBreakSensorOne.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorTwo.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorThree.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorFour.enableLimitSwitch(false));

    return errors == 0;
  }

  private void prepareToShoot() {
    frontMotor.set(ConveyerCal.PREPARE_TO_SHOOT_FRONT_SPEED);
    backMotor.set(ConveyerCal.PREPARE_TO_SHOOT_BACK_SPEED);
  }

  private void scoreTrapOrAmp() {
    frontMotor.set(ConveyerCal.SCORE_AMP_TRAP_FRONT_SPEED);
    backMotor.set(ConveyerCal.SCORE_AMP_TRAP_BACK_SPEED);
  }

  private void receive() {
    frontMotor.set(ConveyerCal.FRONT_HOLD_SPEED);
    backMotor.set(0.0);
  }

  private void stop() {
    frontMotor.set(0.0);
    backMotor.set(0.0);
  }

  @Override
  public void periodic() {
    if (desiredNotePosition == currentNotePosition) {
      stop();
    } else {
      switch (desiredNotePosition) {
        case NO_NOTE:
          stop();
          break;
        case HOLDING_NOTE:
          receive();
          break;
        case PARTIAL_NOTE:
          switch (desiredAction) {
            case NO_ACTION:
              stop();
              break;
            case SHOOT:
              prepareToShoot();
              break;
            case TRAP_AMP:
              scoreTrapOrAmp();
              break;
            case RECEIVE:
              receive();
              break;
          }
          break;
      }

    }
  }
}

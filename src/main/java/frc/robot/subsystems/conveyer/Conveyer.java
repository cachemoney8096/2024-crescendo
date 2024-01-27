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
  private SparkLimitSwitch beamBreakSensorOne,
      beamBreakSensorTwo,
      beamBreakSensorThree,
      beamBreakSensorFour;

  public enum NotePosition {
    NO_NOTE,
    RECEIVING_NOTE,
    HELD_NOTE
  }

  public NotePosition notePosition = NotePosition.NO_NOTE;

  public double secondMotorZeroPosition;

  public Conveyer() {
    SparkMaxUtils.initWithRetry(this::setUpConveyerSparks, ConveyerCal.SPARK_INIT_RETRY_ATTEMPTS);

    beamBreakSensorOne = frontMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    beamBreakSensorTwo = frontMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    beamBreakSensorThree = backMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    beamBreakSensorFour = backMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    beamBreakSensorOne.enableLimitSwitch(false);
    beamBreakSensorTwo.enableLimitSwitch(false);
    beamBreakSensorThree.enableLimitSwitch(false);
    beamBreakSensorFour.enableLimitSwitch(false);

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

    return errors == 0;
  }

  public void prepareToShoot() {
    frontMotor.set(ConveyerCal.PREPARE_TO_SHOOT_FRONT_SPEED);
    backMotor.set(ConveyerCal.PREPARE_TO_SHOOT_BACK_SPEED);
  }

  public void scoreTrapOrAmp() {
    frontMotor.set(ConveyerCal.SCORE_AMP_TRAP_FRONT_SPEED);
    backMotor.set(ConveyerCal.SCORE_AMP_TRAP_BACK_SPEED);
  }

  public void hold() {
    frontMotor.set(ConveyerCal.FRONT_HOLD_SPEED);
    backMotor.set(ConveyerCal.BACK_HOLD_SPEED);
  }

  public void stop() {
    frontMotor.set(0.0);
    backMotor.set(0.0);
  }

  public void startReceivingNote() {
    notePosition = NotePosition.RECEIVING_NOTE;
    secondMotorZeroPosition = backMotorEncoder.getPosition();
    hold();
  }

  /**
   * Get the current position of the note. While we have the beam break sensors available, we are
   * attempting to not use them, and rather just check the note position with the motors.
   */
  public NotePosition getNotePosition() {
    /**
     * If the front motor is not moving and we currently have no note, we are not receiving a note
     */
    if (notePosition == NotePosition.NO_NOTE && frontMotorEncoder.getVelocity() < 1) {
      notePosition = NotePosition.NO_NOTE;
    } else {
      /** If the back motor has moved more than the threshold, we have a note */
      if (backMotorEncoder.getPosition() - secondMotorZeroPosition
          > ConveyerCal.NOTE_THRESHOLD_DEGREES) {
        notePosition = NotePosition.HELD_NOTE;
      } else {
        /**
         * If we were receiving a note and the back motor has not moved more than the threshold, we
         * are no still receiving a note
         */
        notePosition = NotePosition.RECEIVING_NOTE;
      }
    }

    return notePosition;
  }
}

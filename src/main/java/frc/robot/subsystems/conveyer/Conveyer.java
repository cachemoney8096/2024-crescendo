package frc.robot.subsystems.conveyer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

  public ConveyerPositions currentNotePosition = ConveyerPositions.NO_NOTE;

  private double backMotorZeroPosition;

  public Conveyer() {
    SparkMaxUtils.initWithRetry(this::setUpConveyerSparks, ConveyerCal.SPARK_INIT_RETRY_ATTEMPTS);
    backMotorZeroPosition = backMotorEncoder.getPosition();
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
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(frontMotorEncoder, ConveyerConstants.FRONT_GEAR_RATIO));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(backMotorEncoder, ConveyerConstants.BACK_GEAR_RATIO));

    errors += SparkMaxUtils.check(beamBreakSensorOne.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorTwo.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorThree.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorFour.enableLimitSwitch(false));

    return errors == 0;
  }

  public Command shoot() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> frontMotor.set(ConveyerCal.PREPARE_TO_SHOOT_FRONT_SPEED)),
      new InstantCommand(() -> backMotor.set(ConveyerCal.PREPARE_TO_SHOOT_BACK_SPEED)),
      new InstantCommand(() -> currentNotePosition = ConveyerPositions.PARTIAL_NOTE),
      new WaitCommand(ConveyerCal.NOTE_EXIT_TIME_SHOOTER_SECONDS),
      stop(),
      new InstantCommand(() -> currentNotePosition = ConveyerPositions.NO_NOTE),
      new WaitUntilCommand(() -> backMotorEncoder.getVelocity() < ConveyerCal.MOTOR_VELOCITY_THRESHOLD_RPM),
      new InstantCommand(() -> backMotorZeroPosition = backMotorEncoder.getPosition())
    );
  }

  public Command scoreTrapOrAmp() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> frontMotor.set(ConveyerCal.SCORE_AMP_TRAP_FRONT_SPEED)),
      new InstantCommand(() -> backMotor.set(ConveyerCal.SCORE_AMP_TRAP_BACK_SPEED)),
      new InstantCommand(() -> currentNotePosition = ConveyerPositions.PARTIAL_NOTE),
      new WaitCommand(ConveyerCal.NOTE_EXIT_TIME_TRAP_AMP_SECONDS),
      stop(),
      new InstantCommand(() -> currentNotePosition = ConveyerPositions.NO_NOTE),
      new WaitUntilCommand(() -> backMotorEncoder.getVelocity() < ConveyerCal.MOTOR_VELOCITY_THRESHOLD_RPM),
      new InstantCommand(() -> backMotorZeroPosition = backMotorEncoder.getPosition())
    );
  }

  public Command receive() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> frontMotor.set(ConveyerCal.FRONT_RECEIVE_SPEED)),
      new InstantCommand(() -> backMotor.set(0.0)),
      new InstantCommand(() -> currentNotePosition = ConveyerPositions.PARTIAL_NOTE),
      new WaitUntilCommand(() -> Math.abs(backMotorEncoder.getPosition() - backMotorZeroPosition) < ConveyerCal.NOTE_POSITION_THRESHOLD_DEGREES),
      new InstantCommand(() -> backMotorZeroPosition = backMotorEncoder.getPosition()),
      new InstantCommand(() -> frontMotor.set(ConveyerCal.BACK_OFF_POWER)),
      new InstantCommand(() -> backMotor.set(ConveyerCal.BACK_OFF_POWER)),
      new WaitUntilCommand(() -> Math.abs(backMotorEncoder.getPosition() - backMotorZeroPosition) < ConveyerCal.BACK_OFF_DEGREES),
      stop(),
      new InstantCommand(() -> currentNotePosition = ConveyerPositions.HOLDING_NOTE)

    );
  }

  private Command stop() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> frontMotor.set(0.0)),
      new InstantCommand(() -> backMotor.set(0.0))
    );
  }
}

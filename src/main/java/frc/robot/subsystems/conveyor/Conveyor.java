package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;

public class Conveyor extends SubsystemBase {
  public CANSparkMax frontMotor =
      new CANSparkMax(RobotMap.FRONT_CONVEYOR_CAN_ID, MotorType.kBrushless);
  public CANSparkMax backMotor =
      new CANSparkMax(RobotMap.BACK_CONVEYOR_CAN_ID, MotorType.kBrushless);

  /** Configured to read inches, positive towards the shooter. */
  private RelativeEncoder frontMotorEncoder = frontMotor.getEncoder();
  /** Configured to read inches, positive towards the shooter. */
  private RelativeEncoder backMotorEncoder = backMotor.getEncoder();

  /** False is blocked (i.e. there is a note) */
  public DigitalInput intakeBeamBreakSensor = new DigitalInput(RobotMap.INTAKE_BEAM_BREAK_DIO);

  /**
   * These aren't actually limit switches; we just use SparkLimitSwitch objects to access them
   * easily. At the moment, these are unused. TODO: If we use these, add them to Shuffleboard.
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
   *   <li>NO_NOTE: There is no note in the conveyor.
   *   <li>HOLDING_NOTE: The conveyor is holding a note in place.
   *   <li>PARTIAL_NOTE: The conveyor is in the process of receiving a note from the intake, sending
   *       a note to the shooter, or scoring a note in the trap or amp.
   * </ul>
   */
  public enum ConveyorPosition {
    /** There is no note in the conveyor. */
    NO_NOTE,
    /** The conveyor is holding a note in place. */
    HOLDING_NOTE,
    /**
     * The conveyor is in the process of receiving a note from the intake, sending a note to the
     * shooter, or scoring a note in the trap or amp.
     */
    PARTIAL_NOTE
  }

  private ConveyorPosition currentNotePosition = ConveyorPosition.NO_NOTE;

  Command rumbleCommand;

  public Conveyor(Command rumbleCommand) {
    SparkMaxUtils.initWithRetry(this::setUpConveyorSparks, ConveyorCal.SPARK_INIT_RETRY_ATTEMPTS);
    this.rumbleCommand = rumbleCommand;
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean setUpConveyorSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(frontMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(backMotor.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(frontMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(backMotor.setIdleMode(IdleMode.kCoast));

    frontMotor.setInverted(ConveyorConstants.FRONT_MOTOR_INVERTED);
    backMotor.setInverted(ConveyorConstants.BACK_MOTOR_INVERTED);

    errors +=
        SparkMaxUtils.check(
            frontMotor.setSmartCurrentLimit(ConveyorCal.CONVEYOR_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            backMotor.setSmartCurrentLimit(ConveyorCal.CONVEYOR_CURRENT_LIMIT_AMPS));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setLinearFromGearRatio(
                frontMotorEncoder,
                ConveyorConstants.FRONT_GEAR_RATIO,
                ConveyorConstants.CONVEYOR_MOTOR_ROLLER_DIAMETER_IN));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setLinearFromGearRatio(
                backMotorEncoder,
                ConveyorConstants.BACK_GEAR_RATIO,
                ConveyorConstants.CONVEYOR_MOTOR_ROLLER_DIAMETER_IN));

    errors += SparkMaxUtils.check(beamBreakSensorOne.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorTwo.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorThree.enableLimitSwitch(false));
    errors += SparkMaxUtils.check(beamBreakSensorFour.enableLimitSwitch(false));

    backMotorEncoder.setPosition(0.0);
    frontMotorEncoder.setPosition(0.0);

    return errors == 0;
  }

  public void burnFlashSparks() {
    Timer.delay(0.005);
    frontMotor.burnFlash();
    Timer.delay(0.005);
    backMotor.burnFlash();
  }

  public void stopRollers() {
    frontMotor.set(0.0);
    backMotor.set(0.0);
  }

  public void startBackRollers(double power) {
    backMotor.set(power);
  }

  public void stopBackRollers() {
    backMotor.set(0.0);
  }

  public void startFrontRollers(double power) {
    frontMotor.set(power);
  }

  public void stopFrontRollers() {
    frontMotor.set(0.0);
  }

  public void startRollers(double power) {
    frontMotor.set(power);
    backMotor.set(power);
  }

  /** Send note to the shooter. */
  public static Command shoot(Conveyor conveyor) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> conveyor.frontMotor.set(ConveyorCal.PREPARE_TO_SHOOT_FRONT_SPEED), conveyor),
        new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.PREPARE_TO_SHOOT_BACK_SPEED)),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.PARTIAL_NOTE),
        new WaitCommand(ConveyorCal.NOTE_EXIT_TIME_SHOOTER_SECONDS),
        Conveyor.stop(conveyor),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.NO_NOTE));
  }

  public static Command backUpNote(Conveyor conveyor) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> conveyor.backMotorEncoder.setPosition(0.0), conveyor),
        new InstantCommand(() -> conveyor.frontMotor.set(ConveyorCal.FRONT_BACKUP_SPEED)),
        new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.FRONT_BACKUP_SPEED)),
        new WaitCommand(0.25),
        Conveyor.stop(conveyor),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.HOLDING_NOTE));
  }

  /** Score note into the trap or the amp */
  public static Command scoreTrapOrAmp(Conveyor conveyor) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> conveyor.frontMotor.set(ConveyorCal.SCORE_TRAP_FRONT_SPEED), conveyor),
        new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.SCORE_TRAP_BACK_SPEED)),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.PARTIAL_NOTE),
        new WaitCommand(ConveyorCal.NOTE_EXIT_TIME_TRAP_AMP_SECONDS),
        Conveyor.stop(conveyor),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.NO_NOTE));
  }

  /** Get a note from the intake. */
  public static Command receive(Conveyor conveyor) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> conveyor.backMotorEncoder.setPosition(0.0), conveyor),
        new InstantCommand(() -> conveyor.frontMotor.set(ConveyorCal.FRONT_RECEIVE_SPEED)),
        new InstantCommand(() -> conveyor.backMotor.set(0.0)),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.PARTIAL_NOTE), 
        new ParallelRaceGroup(
          new WaitUntilCommand(
              () ->
                  Math.abs(conveyor.backMotorEncoder.getPosition())
                      > ConveyorCal.NOTE_POSITION_THRESHOLD_INCHES).andThen(new InstantCommand(() -> System.out.println("Ended because of back conveyor position"))),
          new WaitUntilCommand(() -> !conveyor.intakeBeamBreakSensor.get()).andThen(new WaitCommand(0.25)).andThen(new InstantCommand(() -> System.out.println("Ended because of intake beam break")))
        ),
        conveyor.rumbleCommand,
        new InstantCommand(() -> SmartDashboard.putBoolean("Have Note", true)),
        new InstantCommand(() -> conveyor.frontMotorEncoder.setPosition(0.0), conveyor),
        new InstantCommand(() -> conveyor.frontMotor.set(ConveyorCal.BACK_OFF_POWER)),
        new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.BACK_OFF_POWER)),
        new WaitUntilCommand(
            () -> conveyor.frontMotorEncoder.getPosition() < ConveyorCal.BACK_OFF_INCHES),
        Conveyor.stop(conveyor),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.HOLDING_NOTE));
  }

  /** Stop the conveor rollers. */
  public static Command stop(Conveyor conveyor) {
    return new InstantCommand(conveyor::stopRollers, conveyor);
  }

  /** backs the currently held note a little bit back into the conveyor to crush it */
  public static Command crushNote(Conveyor conveyor) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> conveyor.backMotorEncoder.setPosition(0.0), conveyor),
        new InstantCommand(() -> conveyor.frontMotor.set(ConveyorCal.FRONT_RECEIVE_SPEED)),
        new InstantCommand(() -> conveyor.backMotor.set(0.0)),
        new WaitUntilCommand(
                () ->
                    Math.abs(conveyor.backMotorEncoder.getPosition())
                        > ConveyorCal.NOTE_POSITION_THRESHOLD_INCHES)
            .withTimeout(1.0),
        new WaitCommand(0.25),
        Conveyor.stop(conveyor));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Current Note Position", () -> currentNotePosition.toString(), null);
    builder.addDoubleProperty(
        "Front Motor Position (in)",
        () -> {
          return frontMotorEncoder.getPosition();
        },
        null);
    builder.addDoubleProperty(
        "Back Motor Position (in)",
        () -> {
          return backMotorEncoder.getPosition();
        },
        null);
    builder.addDoubleProperty(
        "Front Motor Velocity (in per sec)",
        () -> {
          return frontMotorEncoder.getVelocity();
        },
        null);
    builder.addDoubleProperty(
        "Back Motor Velocity (in per sec)",
        () -> {
          return backMotorEncoder.getVelocity();
        },
        null);
    builder.addBooleanProperty("Intake Sensor", () -> intakeBeamBreakSensor.get(), null);
  }
}

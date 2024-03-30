package frc.robot.subsystems.conveyor;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeSequence;
import frc.robot.subsystems.lights.Lights;
import frc.robot.utils.SparkMaxUtils;
import java.util.function.DoubleConsumer;

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

  public DigitalInput frontConveyorBeamBreakSensor =
      new DigitalInput(RobotMap.FRONT_CONVEYOR_BEAM_BREAK_DIO);
  public DigitalInput backConveyorBeamBreakSensor =
      new DigitalInput(RobotMap.BACK_CONVEYOR_BEAM_BREAK_DIO);

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

  public DoubleConsumer rumbleSetter;
  public Runnable hasNoteFunc;

  public Conveyor(DoubleConsumer rumbleSetter, Runnable setHasNote) {
    SparkMaxUtils.initWithRetry(this::setUpConveyorSparks, ConveyorCal.SPARK_INIT_RETRY_ATTEMPTS);
    this.rumbleSetter = rumbleSetter;
    this.hasNoteFunc = setHasNote;
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean setUpConveyorSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(frontMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(backMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20));
    errors += SparkMaxUtils.check(frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20));
    errors += SparkMaxUtils.check(frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50));
    errors += SparkMaxUtils.check(frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    errors += SparkMaxUtils.check(frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    errors += SparkMaxUtils.check(frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200));
    errors += SparkMaxUtils.check(frontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200));
    errors += SparkMaxUtils.check(backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20));
    errors += SparkMaxUtils.check(backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20));
    errors += SparkMaxUtils.check(backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50));
    errors += SparkMaxUtils.check(backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    errors += SparkMaxUtils.check(backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    errors += SparkMaxUtils.check(backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200));
    errors += SparkMaxUtils.check(backMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200));
    errors += SparkMaxUtils.check(frontMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(backMotor.setIdleMode(IdleMode.kBrake));

    Timer.delay(0.1);

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
  public static Command shoot(Conveyor conveyor, double waitTime) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> conveyor.frontMotor.set(ConveyorCal.PREPARE_TO_SHOOT_FRONT_SPEED), conveyor),
        new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.PREPARE_TO_SHOOT_BACK_SPEED)),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.PARTIAL_NOTE),
        new WaitCommand(waitTime),
        Conveyor.stop(conveyor),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.NO_NOTE));
  }

  /** Send note to the shooter. */
  public static Command shoot(Conveyor conveyor) {
    return Conveyor.shoot(conveyor, ConveyorCal.NOTE_EXIT_TIME_SHOOTER_TELEOP_SECONDS);
  }

  /** Score note into the trap or the amp */
  public static Command scoreTrapOrAmp(Conveyor conveyor, boolean scoringAmp) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> conveyor.frontMotor.set(ConveyorCal.SCORE_TRAP_FRONT_SPEED), conveyor),
        new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.SCORE_TRAP_BACK_SPEED)),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.PARTIAL_NOTE),
        new WaitCommand(scoringAmp ? ConveyorCal.NOTE_EXIT_TIME_AMP_SECONDS : ConveyorCal.NOTE_EXIT_TIME_TRAP_SECONDS),
        Conveyor.stop(conveyor),
        new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.NO_NOTE));
  }

  /** Get a note from the intake, ends when conveyor sees the note. */
  public static Command startReceive(Conveyor conveyor) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> conveyor.frontMotor.set(ConveyorCal.RECEIVE_SPEED), conveyor),
            new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.RECEIVE_MEDIUM_SPEED)),
            new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.PARTIAL_NOTE),
            new WaitUntilCommand(() -> !conveyor.backConveyorBeamBreakSensor.get()))
        .withName("Start Receive");
  }

  /** Rumbles the controllers. */
  public static Command rumbleBriefly(Conveyor conveyor) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  conveyor.rumbleSetter.accept(1);
                }),
            new WaitCommand(0.25),
            new InstantCommand(
                () -> {
                  conveyor.rumbleSetter.accept(0.25);
                }))
        .finallyDo(() -> conveyor.rumbleSetter.accept(0));
  }

  /**
   * Assuming a note is here, then runs until it's positioned correctly. Notably, it won't do
   * anything if there's not a note.
   */
  public static Command finishReceive(Conveyor conveyor, Lights lights, boolean letGoOfTrigger) {
    return new ConditionalCommand(
            finishReceiveFunctionalCmd(conveyor, lights),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(1.0),
                    new WaitUntilCommand(() -> !conveyor.backConveyorBeamBreakSensor.get())),
                finishReceiveFunctionalCmd(conveyor, lights)),
            () -> !letGoOfTrigger)
        .withName("Finish Receive");
  }

  public static Command finishReceiveFunctionalCmd(Conveyor conveyor, Lights lights) {
    return new ConditionalCommand(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> conveyor.frontMotor.set(ConveyorCal.RECEIVE_SLOW_SPEED), conveyor),
            new InstantCommand(() -> conveyor.backMotor.set(ConveyorCal.RECEIVE_SLOW_SPEED)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> conveyor.frontConveyorBeamBreakSensor.get()),
            Conveyor.stop(conveyor),
            new InstantCommand(() -> conveyor.stopRollers()),
            new InstantCommand(() -> SmartDashboard.putBoolean("Have Note", true)),
            new InstantCommand(() -> conveyor.currentNotePosition = ConveyorPosition.HOLDING_NOTE)),
        new InstantCommand(),
        () -> {
          return !conveyor.frontConveyorBeamBreakSensor.get();
        });
  }

  public static Command finishReceive(Conveyor conveyor, Lights lights) {
    return finishReceive(conveyor, lights, false);
  }

  /** Stop the conveor rollers. */
  public static Command stop(Conveyor conveyor) {
    return new InstantCommand(conveyor::stopRollers, conveyor).ignoringDisable(true);
  }

  /** backs the currently held note a little bit back into the conveyor to crush it */
  public static Command crushNote(Conveyor conveyor) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> conveyor.frontMotor.set(ConveyorCal.RECEIVE_SPEED)),
        new InstantCommand(() -> conveyor.backMotor.set(-ConveyorCal.RECEIVE_SPEED)),
        new InstantCommand(() -> System.out.println("starting crush")),
        new WaitCommand(1.0),
        new InstantCommand(() -> System.out.println("ending crush")),
        Conveyor.stop(conveyor));
  }

  private boolean sawNote = false;

  @Override
  public void periodic() {
    if (!intakeBeamBreakSensor.get()) {
      if (sawNote) {
        // see and previously saw
      } else {
        sawNote = true;
        Conveyor.rumbleBriefly(this).schedule();
        IntakeSequence.gotNote = true;
        hasNoteFunc.run();
      }
    } else {
      if (sawNote) {
        sawNote = false;
      } else {
        // don't see didn't see do nothing
      }
    }
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
    builder.addBooleanProperty(
        "Front conveyor sensor sees note", () -> !frontConveyorBeamBreakSensor.get(), null);
    builder.addBooleanProperty(
        "Back conveyor sensor sees note", () -> !backConveyorBeamBreakSensor.get(), null);
    builder.addBooleanProperty("Intake sensor sees note", () -> !intakeBeamBreakSensor.get(), null);
  }
}

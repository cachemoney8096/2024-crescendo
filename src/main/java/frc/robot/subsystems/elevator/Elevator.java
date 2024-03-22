package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.CRTUtil;
import frc.robot.utils.SendableHelper;
import frc.robot.utils.SparkMaxUtils;
import java.util.Optional;
import java.util.TreeMap;

/**
 * Moves the elevator carriage up and down. Does not control anything on the carriage (that's the
 * conveyor).
 */
public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    HOME,
    SCORE_TRAP,
    SCORE_AMP,
    PRE_CLIMB,
    SLIGHTLY_UP,
    INTAKING
  }

  private CRTUtil crtUtil = CRTUtil.init(26.0, 44.0, 3.45575, 15, 1).get();

  public CANSparkMax leftMotor =
      new CANSparkMax(RobotMap.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
  public CANSparkMax rightMotor =
      new CANSparkMax(RobotMap.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder leftMotorEncoderRel = leftMotor.getEncoder();
  private final RelativeEncoder rightMotorEncoderRel = rightMotor.getEncoder();
  private final AbsoluteEncoder leftMotorEncoderAbs = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder rightMotorEncoderAbs =
      rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private ProfiledPIDController noteScoringElevatorController =
      new ProfiledPIDController(
          ElevatorCal.NOTE_SCORING_P,
          ElevatorCal.NOTE_SCORING_I,
          ElevatorCal.NOTE_SCORING_D,
          new TrapezoidProfile.Constraints(
              ElevatorCal.MAX_VELOCITY_IN_PER_SECOND,
              ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED));
  private ProfiledPIDController chainGrabberElevatorController =
      new ProfiledPIDController(
          ElevatorCal.CLIMBING_P,
          ElevatorCal.CLIMBING_I,
          ElevatorCal.CLIMBING_D,
          new TrapezoidProfile.Constraints(
              ElevatorCal.MAX_VELOCITY_IN_PER_SECOND_CLIMB,
              ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED_CLIMB));

  private boolean currentlyUsingNoteControl = true;
  private ProfiledPIDController currentPIDController = noteScoringElevatorController;
  private SimpleMotorFeedforward currentFeedforward = ElevatorCal.NOTE_SCORING_FF;

  private TreeMap<ElevatorPosition, Double> elevatorPositions;

  private ElevatorPosition desiredPosition = ElevatorPosition.HOME;

  private boolean allowElevatorMovement = false;

  /** FPGA timestamp from previous cycle. Empty for first cycle only. */
  private Optional<Double> prevTimestamp = Optional.empty();

  /** Profiled velocity setpoint from previous cycle (inches per sec) */
  private double prevVelocityInPerSec = 0;

  // Things for debug
  private double elevatorDemandVoltsA = 0.0;
  private double elevatorDemandVoltsB = 0.0;
  private double elevatorDemandVoltsC = 0.0;
  private double desiredSetpointPosition = 0.0;
  private double desiredSetpointVelocity = 0.0;

  public Elevator() {
    SparkMaxUtils.initWithRetry(this::initSparks, ElevatorConstants.MAX_INIT_RETRY_ATTEMPTS);
    elevatorPositions = new TreeMap<ElevatorPosition, Double>();
    elevatorPositions.put(ElevatorPosition.HOME, ElevatorCal.POSITION_HOME_INCHES);
    elevatorPositions.put(ElevatorPosition.SCORE_AMP, ElevatorCal.POSITION_SCORE_AMP_INCHES);
    elevatorPositions.put(ElevatorPosition.SCORE_TRAP, ElevatorCal.POSITION_SCORE_TRAP_INCHES);
    elevatorPositions.put(ElevatorPosition.PRE_CLIMB, ElevatorCal.POSITION_PRE_CLIMB_INCHES);
    elevatorPositions.put(ElevatorPosition.SLIGHTLY_UP, ElevatorCal.POSITION_SLIGHTLY_UP_INCHES);
    elevatorPositions.put(ElevatorPosition.INTAKING, ElevatorCal.POSITION_INTAKE_INCHES);
    setControlParams(true);
  }

  private boolean initSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(leftMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(rightMotor.restoreFactoryDefaults());
    // errors += SparkMaxUtils.check(leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20));
    errors += SparkMaxUtils.check(leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20));
    errors += SparkMaxUtils.check(leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50));
    errors += SparkMaxUtils.check(leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    errors += SparkMaxUtils.check(leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    errors += SparkMaxUtils.check(leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200));
    errors += SparkMaxUtils.check(leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200));
    // errors += SparkMaxUtils.check(rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20));
    errors += SparkMaxUtils.check(rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20));
    errors += SparkMaxUtils.check(rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50));
    errors += SparkMaxUtils.check(rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    errors += SparkMaxUtils.check(rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    errors += SparkMaxUtils.check(rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200));
    errors += SparkMaxUtils.check(rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200));

    errors += SparkMaxUtils.check(leftMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(rightMotor.setIdleMode(IdleMode.kBrake));

    errors +=
        SparkMaxUtils.check(
            leftMotor.setSmartCurrentLimit(ElevatorCal.ELEVATOR_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            rightMotor.setSmartCurrentLimit(ElevatorCal.ELEVATOR_CURRENT_LIMIT_AMPS));

    errors += SparkMaxUtils.check(leftMotorEncoderAbs.setInverted(true));
    rightMotor.setInverted(true);

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setLinearFromGearRatio(
                leftMotorEncoderRel,
                ElevatorConstants.ELEVATOR_GEAR_RATIO,
                ElevatorConstants.ELEVATOR_DRUM_DIAMETER_IN));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setLinearFromGearRatio(
                rightMotorEncoderRel,
                ElevatorConstants.ELEVATOR_GEAR_RATIO,
                ElevatorConstants.ELEVATOR_DRUM_DIAMETER_IN));

    errors += SparkMaxUtils.check(setLeftZeroFromAbsolute());
    errors += SparkMaxUtils.check(setRightZeroFromAbsolute());

    return errors == 0;
  }

  /** Zeroes leftMotorEncoderRel so it returns inches from home. */
  private REVLibError setLeftZeroFromAbsolute() {
    // TODO absolute encoder zeroing
    return leftMotorEncoderRel.setPosition(ElevatorCal.POSITION_HOME_INCHES);
  }

  /** Zeroes rightMotorEncoderRel so it returns inches from home. */
  private REVLibError setRightZeroFromAbsolute() {
    // TODO absolute encoder zeroing
    return rightMotorEncoderRel.setPosition(ElevatorCal.POSITION_HOME_INCHES);
  }

  /** If true, use elevator control parameters for note scoring as opposed to climbing */
  public void setControlParams(boolean useNoteControlParams) {
    currentlyUsingNoteControl = useNoteControlParams;
    if (useNoteControlParams) {
      currentPIDController = noteScoringElevatorController;
      currentFeedforward = ElevatorCal.NOTE_SCORING_FF;
    } else {
      currentPIDController = chainGrabberElevatorController;
      currentFeedforward = ElevatorCal.CLIMBING_FF;
    }
    currentPIDController.reset(leftMotorEncoderRel.getPosition());
  }

  /** True if the elevator is near OR below home. */
  private boolean nearHome() {
    return leftMotorEncoderRel.getPosition() < (elevatorPositions.get(ElevatorPosition.HOME) + 1.0);
  }

  /** Sets elevator motor voltage based on input position. Should be called every cycle. */
  private void controlPosition(double inputPositionInch) {
    currentPIDController.setGoal(inputPositionInch);
    elevatorDemandVoltsA = currentPIDController.calculate(leftMotorEncoderRel.getPosition());
    final double timestamp = Timer.getFPGATimestamp();
    final double nextVelocityInPerSec = currentPIDController.getSetpoint().velocity;
    if (prevTimestamp.isPresent()) {
      elevatorDemandVoltsB =
          currentFeedforward.calculate(
              prevVelocityInPerSec, nextVelocityInPerSec, timestamp - prevTimestamp.get());
    } else {
      elevatorDemandVoltsB = currentFeedforward.calculate(nextVelocityInPerSec);
    }
    elevatorDemandVoltsC =
        currentlyUsingNoteControl ? ElevatorCal.NOTE_SCORING_KS : ElevatorCal.CLIMBING_KS;

    desiredSetpointPosition = currentPIDController.getSetpoint().position;
    desiredSetpointVelocity = currentPIDController.getSetpoint().velocity;

    double voltageToSet = elevatorDemandVoltsA + elevatorDemandVoltsB + elevatorDemandVoltsC;
    if (desiredPosition == ElevatorPosition.HOME && nearHome() && !currentlyUsingNoteControl) {
      voltageToSet = ElevatorCal.CLIMBING_KS;
    }

    leftMotor.setVoltage(voltageToSet);
    rightMotor.setVoltage(voltageToSet);

    prevVelocityInPerSec = nextVelocityInPerSec;
    prevTimestamp = Optional.of(timestamp);
  }

  public void setDesiredPosition(ElevatorPosition inputPosition, boolean useNoteParams) {
    this.desiredPosition = inputPosition;
    setControlParams(useNoteParams);
    this.allowElevatorMovement = true;
  }

  public void dontAllowElevatorMovement() {
    this.allowElevatorMovement = false;
    leftMotor.setVoltage(0.0);
    rightMotor.setVoltage(0.0);
  }

  public boolean atDesiredPosition() {
    double desiredPositionIn = elevatorPositions.get(desiredPosition);
    double currentPositionIn = leftMotorEncoderRel.getPosition();
    double elevatorMarginInches = ElevatorCal.ELEVATOR_MARGIN_INCHES;
    if (desiredPosition == ElevatorPosition.PRE_CLIMB) {
      elevatorMarginInches = 1;
    }
    if (desiredPosition == ElevatorPosition.SCORE_TRAP) {
      elevatorMarginInches = 2;
    }
    return Math.abs(desiredPositionIn - currentPositionIn) < elevatorMarginInches;
  }

  /**
   * @return true if the elevator's current position is less than the elevator-intake interference
   *     zone
   */
  public boolean elevatorBelowInterferenceZone() {
    return leftMotorEncoderRel.getPosition()
        < ElevatorCal.ELEVATOR_INTERFERENCE_THRESHOLD_MINIMUM_INCHES;
  }

  /**
   * @return true if the elevator's current position is greater than the elevator-intake
   *     interference zone
   */
  public boolean elevatorAboveIntakeInterferenceZone() {
    return leftMotorEncoderRel.getPosition()
        > ElevatorCal.ELEVATOR_INTERFERENCE_THRESHOLD_MAXIMUM_INCHES;
  }

  /**
   * @return true if the elevator's current position is towards the top of the elevator-intake
   *     interference zone
   */
  public boolean elevatorAtTopOfIntakeInterferenceZone() {
    return (leftMotorEncoderRel.getPosition()
            < ElevatorCal.ELEVATOR_INTERFERENCE_THRESHOLD_MAXIMUM_INCHES
        && leftMotorEncoderRel.getPosition()
            > (ElevatorCal.ELEVATOR_INTERFERENCE_THRESHOLD_MAXIMUM_INCHES - 3));
  }

  @Override
  public void periodic() {
    if (allowElevatorMovement) {
      controlPosition(elevatorPositions.get(desiredPosition));
    }
  }

  public void burnFlashSparks() {
    Timer.delay(0.005);
    leftMotor.burnFlash();
    Timer.delay(0.005);
    rightMotor.burnFlash();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SendableHelper.addChild(builder, this, currentPIDController, "CurrentElevatorController");
    builder.addDoubleProperty(
        "Left Elevator Position (in)", leftMotorEncoderRel::getPosition, null);
    builder.addDoubleProperty(
        "Right Elevator Position (in)", rightMotorEncoderRel::getPosition, null);
    builder.addDoubleProperty(
        "Left Elevator Vel (in per s)", leftMotorEncoderRel::getVelocity, null);
    builder.addDoubleProperty(
        "Right Elevator Vel (in per s)", rightMotorEncoderRel::getVelocity, null);
    builder.addDoubleProperty(
        "Elevator Left Abs Pos (deg)", leftMotorEncoderAbs::getPosition, null);
    builder.addDoubleProperty(
        "Elevator Right Abs Pos (deg)", rightMotorEncoderAbs::getPosition, null);
    builder.addDoubleProperty(
        "Elevator Desired Position (in)", () -> elevatorPositions.get(desiredPosition), null);
    builder.addBooleanProperty("Elevator at desired position", this::atDesiredPosition, null);
    builder.addBooleanProperty(
        "Elevator above intererence", this::elevatorAboveIntakeInterferenceZone, null);
    builder.addBooleanProperty(
        "Elevator at top of intererence", this::elevatorAtTopOfIntakeInterferenceZone, null);
    builder.addBooleanProperty(
        "Elevator below interference", this::elevatorBelowInterferenceZone, null);
    builder.addDoubleProperty(
        "Elevator Desired Setpoint (in)", () -> desiredSetpointPosition, null);
    builder.addDoubleProperty(
        "Elevator Desired Vel (in per s)", () -> desiredSetpointVelocity, null);
    builder.addDoubleProperty("Demand PID (V)", () -> elevatorDemandVoltsA, null);
    builder.addDoubleProperty("Demand Feedforward (V)", () -> elevatorDemandVoltsB, null);
    builder.addDoubleProperty("Demand Gravity (V)", () -> elevatorDemandVoltsC, null);
    builder.addBooleanProperty("Near home", this::nearHome, null);
    builder.addBooleanProperty(
        "Holding climb home position",
        () -> desiredPosition == ElevatorPosition.HOME && nearHome() && !currentlyUsingNoteControl,
        null);
    builder.addDoubleProperty(
        "CRT calculated position",
        () ->
            crtUtil.getAbsolutePosition(
                leftMotorEncoderAbs.getPosition(), rightMotorEncoderAbs.getPosition()),
        null);
  }
}

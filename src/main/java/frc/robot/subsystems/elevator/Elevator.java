package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
    POST_CLIMB
  }

  public CANSparkMax leftMotor =
      new CANSparkMax(RobotMap.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
  public CANSparkMax rightMotor =
      new CANSparkMax(RobotMap.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder leftMotorEncoderRel = leftMotor.getEncoder();
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
              ElevatorCal.MAX_VELOCITY_IN_PER_SECOND,
              ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED));

  private ProfiledPIDController currentPIDController = noteScoringElevatorController;
  private SimpleMotorFeedforward currentFeedforward = ElevatorCal.NOTE_SCORING_FF;

  private TreeMap<ElevatorPosition, Double> elevatorPositions;

  private ElevatorPosition desiredPosition = ElevatorPosition.HOME;

  /** FPGA timestamp from previous cycle. Empty for first cycle only. */
  private Optional<Double> prevTimestamp = Optional.empty();

  /** Profiled velocity setpoint from previous cycle (inches per sec) */
  private double prevVelocityInPerSec = 0;

  public Elevator() {
    SparkMaxUtils.initWithRetry(this::initSparks, ElevatorConstants.MAX_INIT_RETRY_ATTEMPTS);
    elevatorPositions = new TreeMap<ElevatorPosition, Double>();
    elevatorPositions.put(ElevatorPosition.HOME, ElevatorCal.POSITION_HOME_INCHES);
    elevatorPositions.put(ElevatorPosition.SCORE_AMP, ElevatorCal.POSITION_SCORE_AMP_INCHES);
    elevatorPositions.put(ElevatorPosition.SCORE_TRAP, ElevatorCal.POSITION_SCORE_TRAP_INCHES);
    elevatorPositions.put(ElevatorPosition.PRE_CLIMB, ElevatorCal.POSITION_PRE_CLIMB_INCHES);
    elevatorPositions.put(ElevatorPosition.POST_CLIMB, ElevatorCal.POSITION_POST_CLIMB_INCHES);
  }

  private boolean initSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(leftMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(rightMotor.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(rightMotor.follow(leftMotor, true));

    errors +=
        SparkMaxUtils.check(
            leftMotor.setSoftLimit(
                SoftLimitDirection.kForward, ElevatorCal.ELEVATOR_POSITIVE_LIMIT_INCHES));
    errors += SparkMaxUtils.check(leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true));
    errors +=
        SparkMaxUtils.check(
            leftMotor.setSoftLimit(
                SoftLimitDirection.kReverse, ElevatorCal.ELEVATOR_NEGATIVE_LIMIT_INCHES));
    errors += SparkMaxUtils.check(leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true));

    errors += SparkMaxUtils.check(leftMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(rightMotor.setIdleMode(IdleMode.kBrake));

    errors +=
        SparkMaxUtils.check(
            leftMotor.setSmartCurrentLimit(ElevatorCal.ELEVATOR_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            rightMotor.setSmartCurrentLimit(ElevatorCal.ELEVATOR_CURRENT_LIMIT_AMPS));

    errors += SparkMaxUtils.check(leftMotorEncoderAbs.setInverted(false));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                leftMotorEncoderAbs, ElevatorConstants.ELEVATOR_LEFT_ABSOLUTE_ENCODER_RATIO));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                rightMotorEncoderAbs, ElevatorConstants.ELEVATOR_RIGHT_ABSOLUTE_ENCODER_RATIO));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setLinearFromGearRatio(
                leftMotorEncoderRel,
                ElevatorConstants.ELEVATOR_GEAR_RATIO,
                ElevatorConstants.ELEVATOR_DRUM_DIAMETER_IN));

    errors += SparkMaxUtils.check(setZeroFromAbsolute());

    return errors == 0;
  }

  /** Zeroes leftMotorEncoderRel so it returns inches from home. */
  private REVLibError setZeroFromAbsolute() {
    // TODO absolute encoder zeroing
    return leftMotorEncoderRel.setPosition(0.0);
  }

  /** If true, use elevator control parameters for note scoring as opposed to climbing */
  public void setControlParams(boolean useNoteControlParams) {
    if (useNoteControlParams) {
      currentPIDController = noteScoringElevatorController;
      currentFeedforward = ElevatorCal.NOTE_SCORING_FF;
    } else {
      currentPIDController = chainGrabberElevatorController;
      currentFeedforward = ElevatorCal.CLIMBING_FF;
    }
    currentPIDController.reset(leftMotorEncoderRel.getPosition());
  }

  /** Sets elevator motor voltage based on input position. Should be called every cycle. */
  private void controlPosition(ElevatorPosition pos) {
    currentPIDController.setGoal(elevatorPositions.get(pos));
    double elevatorDemandVolts = currentPIDController.calculate(leftMotorEncoderRel.getPosition());
    final double timestamp = Timer.getFPGATimestamp();
    final double nextVelocityInPerSec = currentPIDController.getSetpoint().velocity;
    if (prevTimestamp.isPresent()) {
      elevatorDemandVolts +=
          currentFeedforward.calculate(
              prevVelocityInPerSec, nextVelocityInPerSec, timestamp - prevTimestamp.get());
    } else {
      elevatorDemandVolts += currentFeedforward.calculate(nextVelocityInPerSec);
    }
    leftMotor.setVoltage(elevatorDemandVolts);

    prevVelocityInPerSec = nextVelocityInPerSec;
    prevTimestamp = Optional.of(timestamp);
  }

  public void setDesiredPosition(ElevatorPosition inputPosition) {
    this.desiredPosition = inputPosition;
  }

  public boolean atDesiredPosition() {
    double desiredPositionIn = elevatorPositions.get(desiredPosition);
    double currentPositionIn = leftMotorEncoderRel.getPosition();
    return Math.abs(desiredPositionIn - currentPositionIn) < ElevatorCal.ELEVATOR_MARGIN_INCHES;
  }

  /** @return true if the elevator's current position is less than its home threshold */
  public boolean elevatorBelowInterferenceThreshold() {
    return leftMotorEncoderRel.getPosition() < ElevatorCal.ELEVATOR_INTERFERENCE_THRESHOLD_INCHES;
  }

  @Override
  public void periodic() {
    controlPosition(desiredPosition);
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
        "Elevator Position (in)",
        leftMotorEncoderRel::getPosition,
        leftMotorEncoderRel::setPosition);
    builder.addDoubleProperty("Elevator Vel (in/s)", leftMotorEncoderRel::getVelocity, null);
    builder.addDoubleProperty(
        "Elevator Left Abs Pos (deg)", leftMotorEncoderAbs::getPosition, null);
    builder.addDoubleProperty(
        "Elevator Right Abs Pos (deg)", rightMotorEncoderAbs::getPosition, null);
    builder.addBooleanProperty("Elevator at desired position", this::atDesiredPosition, null);
  }
}

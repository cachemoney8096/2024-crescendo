package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.SendableHelper;
import frc.robot.utils.SparkMaxUtils;
import java.util.Optional;
import java.util.TreeMap;

/** Intake pivot and rollers. */
public class Intake extends SubsystemBase {

  public final CANSparkMax pivotMotor =
      new CANSparkMax(RobotMap.INTAKE_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  /** Configured to read degrees, zero is down. */
  private final RelativeEncoder pivotRelativeEncoder = pivotMotor.getEncoder();
  /** Configured to read degrees, zero is down. */
  private final AbsoluteEncoder pivotAbsoluteEncoder =
      pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final TalonFX intakeTalonLeft = new TalonFX(RobotMap.INTAKING_LEFT_MOTOR_CAN_ID);
  private final TalonFX intakeTalonRight = new TalonFX(RobotMap.INTAKING_RIGHT_MOTOR_CAN_ID);
  private ProfiledPIDController pivotController =
      new ProfiledPIDController(
          IntakeCal.INTAKE_PIVOT_P,
          IntakeCal.INTAKE_PIVOT_I,
          IntakeCal.INTAKE_PIVOT_D,
          new TrapezoidProfile.Constraints(
              IntakeCal.PIVOT_MAX_VELOCITY_DEG_PER_SECOND,
              IntakeCal.PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED));

  /** FPGA timestamp from previous cycle. Empty for first cycle only. */
  private Optional<Double> lastControlledTime = Optional.empty();
  /** Profiled velocity setpoint from previous cycle (deg per sec) */
  private Optional<Double> prevVelocityDegPerSec = Optional.empty();

  public enum IntakePosition {
    DEPLOYED,
    STOWED
  }

  private TreeMap<IntakePosition, Double> intakePositionMap;
  private IntakePosition desiredPosition = IntakePosition.STOWED;
  private boolean allowIntakeMovement = false;

  // Stuff for debug
  private double intakeDemandVoltsA = 0.0;
  private double intakeDemandVoltsB = 0.0;
  private double intakeDemandVoltsC = 0.0;
  private double desiredSetpointPosition = 0.0;
  private double desiredSetpointVelocity = 0.0;

  public Intake() {
    initPivotMotor();
    initIntakeTalons();
    intakePositionMap = new TreeMap<IntakePosition, Double>();
    intakePositionMap.put(IntakePosition.DEPLOYED, IntakeCal.INTAKE_DEPLOYED_POSITION_DEGREES);
    intakePositionMap.put(IntakePosition.STOWED, IntakeCal.INTAKE_STOWED_POSITION_DEGREES);

    pivotRelativeEncoder.setPosition(getPivotPositionFromAbs());
  }

  public void initPivotMotor() {
    SparkMaxUtils.initWithRetry(this::setUpPivotSpark, Constants.SPARK_INIT_RETRY_ATTEMPTS);
  }

  /** Does all the initialization for the pivot spark, return true on success */
  private boolean setUpPivotSpark() {
    int errors = 0;
    errors += SparkMaxUtils.check(pivotMotor.restoreFactoryDefaults());

    pivotMotor.setInverted(false);

    errors += SparkMaxUtils.check(pivotMotor.setIdleMode(IdleMode.kBrake));

    errors +=
        SparkMaxUtils.check(
            pivotMotor.setSmartCurrentLimit(IntakeCal.PIVOT_MOTOR_CURRENT_LIMIT_AMPS));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                pivotAbsoluteEncoder, IntakeConstants.INPUT_ABS_ENCODER_GEAR_RATIO));

    pivotAbsoluteEncoder.setInverted(true);

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                pivotRelativeEncoder, IntakeConstants.PIVOT_MOTOR_GEAR_RATIO));

    return errors == 0;
  }

  public void burnFlashSpark() {
    Timer.delay(0.005);
    pivotMotor.burnFlash();
  }

  public void initIntakeTalons() {
    // TODO check status codes
    TalonFXConfigurator cfgLeft = intakeTalonLeft.getConfigurator();
    TalonFXConfigurator cfgRight = intakeTalonRight.getConfigurator();

    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // this is invert = true
    toApply.CurrentLimits.SupplyCurrentLimit = IntakeCal.INTAKING_TALONS_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    cfgLeft.apply(toApply);
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // this is don't invert
    cfgRight.apply(toApply);

    intakeTalonLeft.getDutyCycle().setUpdateFrequency(50);
    intakeTalonRight.getDutyCycle().setUpdateFrequency(50);

    intakeTalonLeft.optimizeBusUtilization();
    intakeTalonRight.optimizeBusUtilization();
  }

  /**
   * @return the difference between the current time and the last controlled time by the timer. If
   *     the lastControlledTime is empty (first record), then return 20 milliseconds
   */
  public double getTimeDifference() {
    if (lastControlledTime.isEmpty()) {
      lastControlledTime = Optional.of(Timer.getFPGATimestamp());
      return Constants.PERIOD_TIME_SECONDS;
    }

    final double currentTimestamp = Timer.getFPGATimestamp();
    final double timestampDifference = currentTimestamp - lastControlledTime.get();
    lastControlledTime = Optional.of(currentTimestamp);

    return timestampDifference;
  }

  public void setDesiredIntakePosition(IntakePosition pos) {
    this.desiredPosition = pos;
    this.allowIntakeMovement = true;
    pivotController.reset(getPivotPosition());
  }

  public void dontAllowIntakeMovement() {
    this.allowIntakeMovement = false;
    pivotMotor.setVoltage(0.0);
  }

  /** Returns the cosine of the arm angle in degrees off of the horizontal. */
  public double getCosineArmAngle() {
    return Math.cos(
        Units.degreesToRadians(
            getPivotPosition() - IntakeConstants.INTAKE_POSITION_WHEN_HORIZONTAL_DEGREES));
  }

  /** Gets the correctly zeroed position of the pivot. */
  public double getPivotPositionFromAbs() {
    double readingDeg = pivotAbsoluteEncoder.getPosition();
    if (readingDeg < IntakeCal.INTAKE_ABSOLUTE_ENCODER_WRAP_POINT_DEG) {
      readingDeg = readingDeg + IntakeConstants.INPUT_ABS_ENCODER_WRAP_INCREMENT_DEGREES;
    }
    return (readingDeg + IntakeCal.INTAKE_ABSOLUTE_ENCODER_ZERO_OFFSET_DEG) % 360.0;
  }

  public double getPivotPosition() {
    return pivotRelativeEncoder.getPosition();
  }

  public void considerZeroingEncoder() {
    if (Math.abs(getPivotPosition() - getPivotPositionFromAbs())
        > IntakeCal.PIVOT_ENCODER_ZEROING_THRESHOLD_DEG) {
      pivotRelativeEncoder.setPosition(getPivotPositionFromAbs());
      pivotController.reset(pivotRelativeEncoder.getPosition());
    }
  }

  /** Sends the pivot towards the input position. Should be called every cycle. */
  private void controlPosition(double inputPositionDeg) {
    // if (Math.abs(pivotController.getPositionError())
    //     > IntakeCal.PIVOT_PROFILE_REPLANNING_THRESHOLD_DEG) {
    //   pivotController.reset(getPivotPosition());
    // }

    pivotController.setGoal(inputPositionDeg);
    intakeDemandVoltsA = pivotController.calculate(getPivotPosition());
    double currentVelocity = pivotController.getSetpoint().velocity;
    if (prevVelocityDegPerSec.isEmpty()) {
      intakeDemandVoltsB = IntakeCal.INTAKE_PIVOT_FEEDFORWARD.calculate(currentVelocity);
    } else {
      intakeDemandVoltsB =
          IntakeCal.INTAKE_PIVOT_FEEDFORWARD.calculate(
              prevVelocityDegPerSec.get(), currentVelocity, getTimeDifference());
    }
    intakeDemandVoltsC = IntakeCal.ARBITRARY_INTAKE_PIVOT_FEEDFORWARD_VOLTS * getCosineArmAngle();

    double voltageToSet = intakeDemandVoltsA + intakeDemandVoltsB + intakeDemandVoltsC;
    if (deployedDesired() && nearDeployed()) {
      voltageToSet = IntakeCal.INTAKE_HOLD_DEPLOYED_VOLTS;
    }

    pivotMotor.setVoltage(voltageToSet);

    desiredSetpointPosition = pivotController.getSetpoint().position;
    desiredSetpointVelocity = pivotController.getSetpoint().velocity;

    prevVelocityDegPerSec = Optional.of(currentVelocity);
  }

  public boolean atDesiredIntakePosition() {
    return atIntakePosition(desiredPosition);
  }

  public boolean atIntakePosition(IntakePosition pos) {
    double checkPositionDegrees = intakePositionMap.get(pos);
    double intakePositionDegrees = getPivotPosition();
    return Math.abs(intakePositionDegrees - checkPositionDegrees)
        <= IntakeCal.INTAKE_MARGIN_DEGREES;
  }

  public boolean nearDeployed() {
    double checkPositionDegrees = intakePositionMap.get(IntakePosition.DEPLOYED);
    double intakePositionDegrees = getPivotPosition();
    return (intakePositionDegrees - checkPositionDegrees) > -4.0;
  }

  public boolean deployedDesired() {
    return desiredPosition == IntakePosition.DEPLOYED;
  }

  /**
   * 90 degrees represents when the intake is at the shooter therefore, being clear of the conveyor
   * zone would be greater than the threshold
   */
  public boolean clearOfConveyorZone() {
    return getPivotPosition() > IntakeCal.CONVEYOR_ZONE_THRESHOLD_DEGREES;
  }

  public void startRollers() {
    intakeTalonLeft.set(IntakeCal.INTAKING_POWER);
    intakeTalonRight.set(IntakeCal.INTAKING_POWER);
  }

  public void stopRollers() {
    intakeTalonLeft.stopMotor();
    intakeTalonRight.stopMotor();
  }

  public void reverseRollers() {
    intakeTalonLeft.set(IntakeCal.REVERSE_INTAKING_POWER);
    intakeTalonRight.set(IntakeCal.REVERSE_INTAKING_POWER);
  }

  public void periodic() {
    if (allowIntakeMovement) {
      controlPosition(intakePositionMap.get(desiredPosition));
    }
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SendableHelper.addChild(builder, this, pivotController, "PivotController");
    builder.addStringProperty("Pivot desired pos", () -> desiredPosition.toString(), null);
    builder.addDoubleProperty(
        "Pivot desired pos (deg)", () -> intakePositionMap.get(desiredPosition), null);
    builder.addDoubleProperty("Pivot abs pos (deg)", pivotAbsoluteEncoder::getPosition, null);
    builder.addDoubleProperty("Pivot abs offset pos (deg)", this::getPivotPositionFromAbs, null);
    builder.addDoubleProperty("Pivot pos (deg)", pivotRelativeEncoder::getPosition, null);
    builder.addDoubleProperty("Pivot Vel (deg)", pivotRelativeEncoder::getVelocity, null);
    builder.addDoubleProperty("Pivot setpoint pos (deg)", () -> desiredSetpointPosition, null);
    builder.addDoubleProperty("Pivot setpoint Vel (deg)", () -> desiredSetpointVelocity, null);
    builder.addDoubleProperty(
        "Pivot setpoint error (deg)",
        () -> {
          double val = pivotController.getPositionError();
          if (Math.abs(val) > 30.0) {
            val = 0.0;
          }
          return val;
        },
        null);
    builder.addBooleanProperty("intake at desired pos", this::atDesiredIntakePosition, null);
    builder.addDoubleProperty("roller power in [-1,1]", intakeTalonLeft::get, null);
    builder.addDoubleProperty("Demand PID (V)", () -> intakeDemandVoltsA, null);
    builder.addDoubleProperty("Demand FF (V)", () -> intakeDemandVoltsB, null);
    builder.addDoubleProperty("Demand Gravity (V)", () -> intakeDemandVoltsC, null);
    builder.addDoubleProperty(
        "Demand Total (V)",
        () -> {
          return intakeDemandVoltsA + intakeDemandVoltsB + intakeDemandVoltsC;
        },
        null);
    builder.addDoubleProperty(
        "Motor Set Speed",
        () -> {
          return pivotMotor.get();
        },
        null);
    builder.addDoubleProperty(
        "Motor Out -1 to 1",
        () -> {
          return pivotMotor.getAppliedOutput();
        },
        null);
    builder.addDoubleProperty(
        "Bus Voltage at spark",
        () -> {
          return pivotMotor.getBusVoltage();
        },
        null);
    builder.addDoubleProperty(
        "Motor Current (A)",
        () -> {
          return pivotMotor.getOutputCurrent();
        },
        null);
    builder.addBooleanProperty("clear of conveyor", this::clearOfConveyorZone, null);
    builder.addBooleanProperty("near deployed", () -> nearDeployed(), null);
    builder.addBooleanProperty("deployed desired", () -> deployedDesired(), null);
  }
}

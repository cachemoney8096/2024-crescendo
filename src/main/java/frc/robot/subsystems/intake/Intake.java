package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
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

  private final CANSparkMax pivotMotor =
      new CANSparkMax(RobotMap.INTAKE_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  /** Configured to read degrees, zero is down. */
  private final AbsoluteEncoder pivotAbsoluteEncoder =
      pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final TalonFX intakeTalonFront = new TalonFX(RobotMap.INTAKING_FRONT_MOTOR_CAN_ID);
  private final TalonFX intakeTalonBack = new TalonFX(RobotMap.INTAKING_BACK_MOTOR_CAN_ID);
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
    STOWED,
    CLEAR_OF_CONVEYOR
  }

  private TreeMap<IntakePosition, Double> intakePositionMap;
  private IntakePosition desiredPosition = IntakePosition.STOWED;

  public Intake() {
    initPivotMotor();
    initIntakeTalons();
    intakePositionMap = new TreeMap<IntakePosition, Double>();
    intakePositionMap.put(IntakePosition.DEPLOYED, IntakeCal.INTAKE_DEPLOYED_POSITION_DEGREES);
    intakePositionMap.put(IntakePosition.STOWED, IntakeCal.INTAKE_STOWED_POSITION_DEGREES);
    intakePositionMap.put(IntakePosition.CLEAR_OF_CONVEYOR, IntakeCal.INTAKE_SAFE_POSITION_DEGREES);
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

    return errors == 0;
  }

  public void initIntakeTalons() {
    // TODO check status codes
    TalonFXConfigurator cfgFront = intakeTalonFront.getConfigurator();
    TalonFXConfigurator cfgBack = intakeTalonFront.getConfigurator();

    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.CurrentLimits.SupplyCurrentLimit = IntakeCal.INTAKING_TALONS_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    cfgFront.apply(toApply);
    cfgBack.apply(toApply);

    intakeTalonBack.setControl(new Follower(intakeTalonFront.getDeviceID(), false));
  }

  /**
   * @return the difference between the current time and the last controlled time by the timer. If
   *     the lastControlledTime is empty (first record), then return 20 milliseconds
   */
  public double getTimeDifference() {
    if (lastControlledTime.isEmpty()) {
      return Constants.PERIOD_TIME_SECONDS;
    }

    final double currentTimestamp = Timer.getFPGATimestamp();
    final double timestampDifference = currentTimestamp - lastControlledTime.get();
    lastControlledTime = Optional.of(currentTimestamp);

    return timestampDifference;
  }

  public void setDesiredIntakePosition(IntakePosition pos) {
    this.desiredPosition = pos;
  }

  /** Returns the cosine of the arm angle in degrees off of the horizontal. */
  public double getCosineArmAngle() {
    return Math.cos(
        Units.degreesToRadians(
            pivotAbsoluteEncoder.getPosition()
                - IntakeConstants.INTAKE_POSITION_WHEN_HORIZONTAL_DEGREES));
  }

  /** Gets the correctly zeroed position of the pivot. */
  public double getOffsetAbsPositionDeg() {
    return pivotAbsoluteEncoder.getPosition() + IntakeCal.INTAKE_ABSOLUTE_ENCODER_ZERO_OFFSET_DEG;
  }

  /** Sends the pivot towards the input position. Should be called every cycle. */
  private void moveToPos(IntakePosition pos) {
    pivotController.setGoal(intakePositionMap.get(pos));
    double intakeDemandVoltsA = pivotController.calculate(getOffsetAbsPositionDeg());
    double intakeDemandVoltsB;
    double currentVelocity = pivotController.getSetpoint().velocity;
    if (prevVelocityDegPerSec.isEmpty()) {
      intakeDemandVoltsB = IntakeCal.INTAKE_PIVOT_FEEDFORWARD.calculate(currentVelocity);
    } else {
      intakeDemandVoltsB =
          IntakeCal.INTAKE_PIVOT_FEEDFORWARD.calculate(
              prevVelocityDegPerSec.get(), currentVelocity, getTimeDifference());
    }
    double intakeDemandVoltsC =
        IntakeCal.ARBITRARY_INTAKE_PIVOT_FEEDFORWARD_VOLTS * getCosineArmAngle();

    pivotMotor.setVoltage(intakeDemandVoltsA + intakeDemandVoltsB + intakeDemandVoltsC);

    prevVelocityDegPerSec = Optional.of(currentVelocity);
  }

  public boolean atDesiredIntakePosition() {
    return atIntakePosition(desiredPosition);
  }

  public boolean atIntakePosition(IntakePosition pos) {
    double checkPositionDegrees = intakePositionMap.get(pos);
    double intakePositionDegrees = pivotAbsoluteEncoder.getPosition();
    return Math.abs(intakePositionDegrees - checkPositionDegrees)
        <= IntakeCal.INTAKE_MARGIN_DEGREES;
  }

  /**
   * 90 degrees represents when the intake is at the shooter therefore, being clear of the conveyor
   * zone would be greater than the threshold
   */
  public boolean clearOfConveyorZone() {
    return pivotAbsoluteEncoder.getPosition() > IntakeCal.CONVEYOR_ZONE_THRESHOLD_DEGREES;
  }

  public void startRollers() {
    intakeTalonFront.set(IntakeCal.INTAKING_POWER);
  }

  public void stopRollers() {
    intakeTalonFront.stopMotor();
  }

  public void reverseRollers() {
    intakeTalonFront.set(IntakeCal.REVERSE_INTAKING_POWER);
  }

  public void periodic() {
    moveToPos(desiredPosition);
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SendableHelper.addChild(builder, this, pivotController, "PivotController");
    builder.addStringProperty("pivot desired pos", () -> desiredPosition.toString(), null);
    builder.addDoubleProperty(
        "pivot desired pos (deg)", () -> intakePositionMap.get(desiredPosition), null);
    builder.addDoubleProperty("pivot abs pos (deg)", pivotAbsoluteEncoder::getPosition, null);
    builder.addDoubleProperty("pivot abs offset pos (deg)", this::getOffsetAbsPositionDeg, null);

    builder.addDoubleProperty(
        "Intake Abs Offset Position (deg)", this::getOffsetAbsPositionDeg, null);

    builder.addDoubleProperty(
        "pivot abs deploy velocity (deg/sec)", pivotAbsoluteEncoder::getVelocity, null);

    builder.addBooleanProperty("intake at desired pos", this::atDesiredIntakePosition, null);

    builder.addDoubleProperty("roller power in [-1,1]", intakeTalonFront::get, null);
  }
}

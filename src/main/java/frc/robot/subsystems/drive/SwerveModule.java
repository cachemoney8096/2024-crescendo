package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModule implements Sendable {
  public final TalonFX drivingTalon;
  public final CANSparkMax turningSparkMax;

  private final RelativeEncoder turningRelativeEncoder;
  private final AbsoluteEncoder turningAbsoluteEncoder;
  private AbsoluteEncoderChecker turningAbsoluteEncoderChecker = new AbsoluteEncoderChecker();

  private final SparkPIDController turningPIDController;

  private double chassisAngularOffsetRadians = 0;

  public TalonFXConfiguration appliedConfiguration;

  /** Desired velocity and angle. This angle includes the chassis offset. */
  public SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor, encoder, and PID
   * controller.
   */
  public SwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingTalon = new TalonFX(drivingCanId);
    turningSparkMax = new CANSparkMax(turningCanId, MotorType.kBrushless);
    chassisAngularOffsetRadians = chassisAngularOffset;

    initDriveTalon();
    SparkMaxUtils.initWithRetry(this::initTurnSpark, DriveCal.SPARK_INIT_RETRY_ATTEMPTS);

    turningRelativeEncoder = turningSparkMax.getEncoder();
    turningAbsoluteEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    turningPIDController = turningSparkMax.getPIDController();

    // desiredState.angle = Rotation2d.fromRadians(turningAbsoluteEncoder.getPosition());
    desiredState.angle = Rotation2d.fromRadians(turningRelativeEncoder.getPosition());
    drivingTalon.setPosition(0);
  }

  /** Does all the initialization for the spark, return true on success */
  boolean initTurnSpark() {
    int errors = 0;

    errors += SparkMaxUtils.check(turningSparkMax.restoreFactoryDefaults());

    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 300));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500));
    errors +=
        SparkMaxUtils.check(turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 500));
    Timer.delay(0.1);

    turningSparkMax.setInverted(false);

    RelativeEncoder tunringRelativeEncoderTmp = turningSparkMax.getEncoder();
    AbsoluteEncoder turningAbsoluteEncoderTmp = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    SparkPIDController turningPidTmp = turningSparkMax.getPIDController();

    errors += SparkMaxUtils.check(turningPidTmp.setFeedbackDevice(tunringRelativeEncoderTmp));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setRadsFromGearRatio(
                tunringRelativeEncoderTmp, DriveConstants.TURN_MODULE_RELATIVE_ENCODER_GEAR_RATIO));

    // Gear ratio 1.0 because the encoder is 1:1 with the module (doesn't involve the actual turning
    // gear ratio)
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setRadsFromGearRatio(
                turningAbsoluteEncoderTmp, DriveConstants.TURN_MODULE_ABSOLUTE_ENCODER_GEAR_RATIO));

    errors +=
        SparkMaxUtils.check(
            turningAbsoluteEncoderTmp.setInverted(DriveConstants.TURNING_ENCODER_INVERTED));

    errors += SparkMaxUtils.check(turningPidTmp.setPositionPIDWrappingEnabled(true));
    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setPositionPIDWrappingMinInput(
                DriveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS));
    errors +=
        SparkMaxUtils.check(
            turningPidTmp.setPositionPIDWrappingMaxInput(
                DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS));

    errors += SparkMaxUtils.check(turningPidTmp.setP(DriveCal.TURNING_P));
    errors += SparkMaxUtils.check(turningPidTmp.setI(DriveCal.TURNING_I));
    errors += SparkMaxUtils.check(turningPidTmp.setD(DriveCal.TURNING_D));
    errors += SparkMaxUtils.check(turningPidTmp.setFF(DriveCal.TURNING_FF));

    errors +=
        SparkMaxUtils.check(turningSparkMax.setIdleMode(DriveConstants.TURNING_MOTOR_IDLE_MODE));
    errors +=
        SparkMaxUtils.check(
            turningSparkMax.setSmartCurrentLimit(DriveConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  /** Does all the initialization for the spark, return true on success */
  void initDriveTalon() {
    // TODO check status codes
    TalonFXConfigurator cfg = drivingTalon.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: check this
    toApply.Feedback.SensorToMechanismRatio =
        DriveConstants.DRIVING_MOTOR_REDUCTION / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    toApply.CurrentLimits.SupplyCurrentLimit =
        DriveConstants.DRIVING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.CurrentLimits.StatorCurrentLimit =
        DriveConstants.DRIVING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.Slot0.kP = DriveCal.DRIVING_P;
    toApply.Slot0.kI = DriveCal.DRIVING_I;
    toApply.Slot0.kD = DriveCal.DRIVING_D;
    toApply.Slot0.kV = DriveCal.DRIVING_FF;
    appliedConfiguration = toApply;
    cfg.apply(toApply);
    final double fastUpdateFrequencyHz = 50.0; // TODO change to faster for better odometry
    final double slowUpdateFrequencyHz = 50.0;
    drivingTalon.getPosition().setUpdateFrequency(fastUpdateFrequencyHz);
    drivingTalon.getVelocity().setUpdateFrequency(fastUpdateFrequencyHz);
    drivingTalon.getClosedLoopProportionalOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopDerivativeOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopIntegratedOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopFeedForward().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.optimizeBusUtilization();
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    turningSparkMax.burnFlash();
  }

  public void considerZeroingEncoder() {
    if (Math.abs(turningAbsoluteEncoder.getPosition()) < 0.01) {
      return;
    }
    if (Math.abs(getEncoderRelativePositionRad() - getEncoderAbsPositionRad())
        > DriveCal.TURNING_ENCODER_ZEROING_THRESHOLD_RAD) {
      turningRelativeEncoder.setPosition(getEncoderAbsPositionRad() - chassisAngularOffsetRadians);
      turningPIDController.setReference(getEncoderRelativePositionRad(), ControlType.kPosition);
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // return new SwerveModuleState(
    //     drivingTalon.getVelocity().getValue(),
    //     new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffsetRadians));
    return new SwerveModuleState(
        drivingTalon.getVelocity().getValue(),
        new Rotation2d(turningRelativeEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // return new SwerveModulePosition(
    //     drivingTalon.getPosition().getValue(),
    //     new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffsetRadians));
    return new SwerveModulePosition(
        drivingTalon.getPosition().getValue(),
        new Rotation2d(turningRelativeEncoder.getPosition()));
  }

  /** Applies slew rate. */
  public double getDesiredVelocityMps(double inputVelocityMps) {
    // Allow any decrease in desired speed
    final double prevDesiredVelocityMps = desiredState.speedMetersPerSecond;
    if (Math.abs(inputVelocityMps) < Math.abs(prevDesiredVelocityMps)) {
      return inputVelocityMps;
    }

    // If the change is less than the max accel, allow it
    final double maxAccelMpss = 15.0;
    final double loopTimeS = 0.02;
    final double maxVelChangeMps = maxAccelMpss * loopTimeS;
    final double velChangeMps = inputVelocityMps - prevDesiredVelocityMps;
    if (Math.abs(velChangeMps) < maxVelChangeMps) {
      return inputVelocityMps;
    }

    // Clamp to max allowed change
    final double allowedChangeMps = MathUtil.clamp(velChangeMps, -maxVelChangeMps, maxVelChangeMps);
    return prevDesiredVelocityMps + allowedChangeMps;
  }

  /** Ensures the value a is in [0, b) */
  public static double mod(double a, double b) {
    double r = a % b;
    if (r < 0) {
      r += b;
    }
    return r;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle. Angle is relative to chassis (no offset
   *     needed).
   */
  public void setDesiredState(SwerveModuleState inputState, boolean overrideSlew) {

    // Optimize the reference state to avoid spinning further than 90 degrees.
    // inputState =
    //     SwerveModuleState.optimize(inputState, new
    // Rotation2d(turningAbsoluteEncoder.getPosition()));
    inputState =
        SwerveModuleState.optimize(
            inputState, new Rotation2d(turningRelativeEncoder.getPosition()));

    // Ensure optimized state
    inputState.angle = Rotation2d.fromRadians(mod(inputState.angle.getRadians(), 2.0 * Math.PI));

    if (!overrideSlew) {
      inputState.speedMetersPerSecond = getDesiredVelocityMps(inputState.speedMetersPerSecond);
    }

    // Setting global desiredState to be optimized for the shuffleboard
    this.desiredState = inputState;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingTalon.setControl(new VelocityDutyCycle(inputState.speedMetersPerSecond).withSlot(0));
    turningPIDController.setReference(
        inputState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
    drivingTalon.setPosition(0);
  }

  public double getEncoderAbsPositionRad() {
    return turningAbsoluteEncoder.getPosition();
  }

  public double getEncoderRelativePositionRad() {
    return turningRelativeEncoder.getPosition();
  }

  public void periodic() {
    turningAbsoluteEncoderChecker.addReading(turningAbsoluteEncoder.getPosition());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Driving kP",
        () -> {
          return DriveCal.DRIVING_P;
        },
        null);
    builder.addDoubleProperty(
        "Driving kI",
        () -> {
          return DriveCal.DRIVING_I;
        },
        null);
    builder.addDoubleProperty(
        "Driving kD",
        () -> {
          return DriveCal.DRIVING_D;
        },
        null);
    builder.addDoubleProperty(
        "Driving kP output contribution",
        () -> {
          return drivingTalon.getClosedLoopProportionalOutput().getValue();
        },
        null); // TODO find a setter? i couldn't
    builder.addDoubleProperty(
        "Driving kI output contribution",
        () -> {
          return drivingTalon.getClosedLoopIntegratedOutput().getValue();
        },
        null);
    builder.addDoubleProperty(
        "Driving kD output contribution",
        () -> {
          return drivingTalon.getClosedLoopDerivativeOutput().getValue();
        },
        null);
    builder.addDoubleProperty(
        "Driving kFF",
        () -> {
          return drivingTalon.getClosedLoopFeedForward().getValue();
        },
        null);
    builder.addDoubleProperty("Turning kP", turningPIDController::getP, turningPIDController::setP);
    builder.addDoubleProperty("Turning kI", turningPIDController::getI, turningPIDController::setI);
    builder.addDoubleProperty("Turning kD", turningPIDController::getD, turningPIDController::setD);
    builder.addDoubleProperty(
        "Turning kFF", turningPIDController::getFF, turningPIDController::setFF);
    builder.addDoubleProperty(
        "Driving Vel (m/s)",
        () -> {
          return drivingTalon.getVelocity().getValue();
        },
        null);
    builder.addDoubleProperty(
        "Steering Pos (rad) - absolute", turningAbsoluteEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Steering Pos (rad) - relative", turningRelativeEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Desired Vel (m/s)",
        () -> {
          return desiredState.speedMetersPerSecond;
        },
        null);
    builder.addDoubleProperty(
        "Desired Steer (rad)",
        () -> {
          return desiredState.angle.getRadians();
        },
        null);
    builder.addBooleanProperty(
        "Turning encoder connected", turningAbsoluteEncoderChecker::encoderConnected, null);
  }
}

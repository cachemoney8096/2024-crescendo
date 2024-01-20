package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModule implements Sendable{
  public final TalonFX drivingTalon;
  public final CANSparkMax turningSparkMax;

  private final AbsoluteEncoder turningEncoder;
  private AbsoluteEncoderChecker turningAbsoluteEncoderChecker = new AbsoluteEncoderChecker();

  private final SparkPIDController turningPIDController;

  private double chassisAngularOffsetRadians = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public SwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingTalon = new TalonFX(drivingCanId);
    turningSparkMax = new CANSparkMax(turningCanId, MotorType.kBrushless);
    chassisAngularOffsetRadians = chassisAngularOffset;

    initDriveTalon();
    SparkMaxUtils.initWithRetry(this::initTurnSpark, DriveCal.SPARK_INIT_RETRY_ATTEMPTS);

    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    turningPIDController = turningSparkMax.getPIDController();

    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingTalon.setPosition(0);
  }

  /** Does all the initialization for the spark, return true on success */
  boolean initTurnSpark() {
    int errors = 0;

    errors += SparkMaxUtils.check(turningSparkMax.restoreFactoryDefaults());
    turningSparkMax.setInverted(false);

    AbsoluteEncoder turningEncoderTmp = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    SparkPIDController turningPidTmp = turningSparkMax.getPIDController();
    errors += SparkMaxUtils.check(turningPidTmp.setFeedbackDevice(turningEncoderTmp));

    // Gear ratio 1.0 because the encoder is 1:1 with the module (doesn't involve the actual turning
    // gear ratio)
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setRadsFromGearRatio(turningEncoderTmp, DriveConstants.TURN_MODULE_ENCODER_GEAR_RATIO));

    errors +=
        SparkMaxUtils.check(
            turningEncoderTmp.setInverted(DriveConstants.TURNING_ENCODER_INVERTED));

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
        SparkMaxUtils.check(
            turningPidTmp.setOutputRange(
                DriveCal.TURNING_MIN_OUTPUT, DriveCal.TURNING_MAX_OUTPUT));

    errors +=
        SparkMaxUtils.check(
            turningSparkMax.setIdleMode(DriveConstants.TURNING_MOTOR_IDLE_MODE));
    errors +=
        SparkMaxUtils.check(
            turningSparkMax.setSmartCurrentLimit(
                DriveConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  /** Does all the initialization for the spark, return true on success */
  void initDriveTalon() {
    TalonFXConfigurator cfg = drivingTalon.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: check this
    toApply.Feedback.SensorToMechanismRatio = 1/DriveConstants.DRIVING_MOTOR_REDUCTION*Math.PI*DriveConstants.WHEEL_DIAMETER_FUDGE_FACTOR; //TODO somebody fix this math
    toApply.CurrentLimits.SupplyCurrentLimit = DriveConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true; 
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.Slot0.kP = DriveCal.DRIVING_P;
    toApply.Slot0.kI = DriveCal.DRIVING_I;
    toApply.Slot0.kD = DriveCal.DRIVING_D;
    toApply.Slot0.kV = DriveCal.DRIVING_FF;
    cfg.apply(toApply);
  }

  /**
   * Burns the current settings to sparks so they keep current settings on reboot. Should be done
   * after all settings are set.
   */
  public void burnFlashSparks() {
    Timer.delay(0.005);
    turningSparkMax.burnFlash();
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        drivingTalon.getVelocity().getValue(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffsetRadians));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingTalon.getPosition().getValue(), 
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffsetRadians));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffsetRadians));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

    // Setting global desiredState to be optimized for the shuffleboard
    this.desiredState = optimizedDesiredState;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingTalon.setControl(new VelocityVoltage(optimizedDesiredState.speedMetersPerSecond)); 
    turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
    drivingTalon.setPosition(0);
  }

  public double getEncoderAbsPositionRad() {
    return turningEncoder.getPosition();
  }

  public void periodic() {
    turningAbsoluteEncoderChecker.addReading(turningEncoder.getPosition());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Driving kP", ()->{return DriveCal.DRIVING_P;}, null);
    builder.addDoubleProperty("Driving kI", ()->{return DriveCal.DRIVING_I;}, null);
    builder.addDoubleProperty("Driving kD", ()->{return DriveCal.DRIVING_D;}, null);
    builder.addDoubleProperty("Driving kP output contribution", ()->{return drivingTalon.getClosedLoopProportionalOutput().getValue();}, null); //TODO find a setter? i couldn't
    builder.addDoubleProperty("Driving kI output contribution", ()->{return drivingTalon.getClosedLoopIntegratedOutput().getValue();}, null);
    builder.addDoubleProperty("Driving kD output contribution", ()->{return drivingTalon.getClosedLoopDerivativeOutput().getValue();}, null);
    builder.addDoubleProperty(
        "Driving kFF", ()->{return drivingTalon.getClosedLoopFeedForward().getValue();}, null); 
    builder.addDoubleProperty("Turning kP", turningPIDController::getP, turningPIDController::setP);
    builder.addDoubleProperty("Turning kI", turningPIDController::getI, turningPIDController::setI);
    builder.addDoubleProperty("Turning kD", turningPIDController::getD, turningPIDController::setD);
    builder.addDoubleProperty(
        "Turning kFF", turningPIDController::getFF, turningPIDController::setFF);
    builder.addDoubleProperty("Driving Vel (m/s)", ()->{ return drivingTalon.getVelocity().getValue();} , null);
    builder.addDoubleProperty("Steering Pos (rad)", turningEncoder::getPosition, null);
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

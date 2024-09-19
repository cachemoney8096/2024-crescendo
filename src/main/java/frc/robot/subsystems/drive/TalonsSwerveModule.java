package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonsSwerveModule extends SubsystemBase {
  public final TalonFX drivingTalon;
  public final TalonFX turningTalon;

  private double chassisAngularOffsetRadians = 0.0;

  public TalonFXConfiguration appliedConfiguration;

  /** desired velocity and angle (degrees?); this angle includes the chassis offset. */
  public SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** if true, set chassis speeds to half the desired value */
  public boolean throttleSpeed = false;

  /**
   * Constructs a SwerveModule and configures the driving and turning motor, encoder, and PID
   * controller.
   */
  public TalonsSwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingTalon = new TalonFX(drivingCanId);
    turningTalon = new TalonFX(turningCanId);
    chassisAngularOffsetRadians = chassisAngularOffset;

    initDriveTalon();
    initTurnTalon();

    // turningPIDController = turningTalon.getPIDController();

    desiredState.angle = Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble());
    drivingTalon.setPosition(0.0);

  } 

  /** does all the initialization for the drive talon
   * @return true on success, false otherwise */
  private void initDriveTalon() {
    // TODO check status codes
    TalonFXConfigurator cfg = drivingTalon.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: based off 2024 robot
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

  /** does all the initialization for the turning talon
   * based on the initDriveTalon() method
   * @return true on success, false otherwise */
  private void initTurnTalon() {
    // TODO check status codes
    TalonFXConfigurator cfg = turningTalon.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: confirm when built
    toApply.Feedback.SensorToMechanismRatio =
        DriveConstants.TURNING_MOTOR_REDUCTION / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
        
    toApply.CurrentLimits.SupplyCurrentLimit =
        DriveConstants.TURNING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.CurrentLimits.StatorCurrentLimit =
        DriveConstants.TURNING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;

    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast; // TODO confirm when built
    toApply.Slot0.kP = DriveCal.TURNING_P;
    toApply.Slot0.kI = DriveCal.TURNING_I;
    toApply.Slot0.kD = DriveCal.TURNING_D;
    toApply.Slot0.kV = DriveCal.TURNING_FF;

    appliedConfiguration = toApply;
    cfg.apply(toApply);

    // TODO check this frequency stuff
    final double fastUpdateFrequencyHz = 50.0; // TODO change to faster for better odometry
    final double slowUpdateFrequencyHz = 50.0;

    turningTalon.getPosition().setUpdateFrequency(fastUpdateFrequencyHz);
    turningTalon.getVelocity().setUpdateFrequency(fastUpdateFrequencyHz);
    turningTalon.getClosedLoopProportionalOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    turningTalon.getClosedLoopDerivativeOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    turningTalon.getClosedLoopIntegratedOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    turningTalon.getClosedLoopFeedForward().setUpdateFrequency(slowUpdateFrequencyHz);
    turningTalon.optimizeBusUtilization();
  }

  // unsure if this method is necessary for this code?
  // public void considerZeroingEncoder() {
  //   if (Math.abs(turningAbsoluteEncoder.getPosition()) < 0.01) {
  //     return;
  //   }
  //   if (Math.abs(getEncoderRelativePositionRad() - getEncoderAbsPositionRad())
  //       > DriveCal.TURNING_ENCODER_ZEROING_THRESHOLD_RAD) {
  //     // turningRelativeEncoder.setPosition(getEncoderAbsPositionRad() - chassisAngularOffsetRadians);
  //     // turningPIDController.setReference(getEncoderRelativePositionRad(), ControlType.kPosition);
  //     turningTalon.setPosition(Rotation2d.fromRadians(getEncoderAbsPositionRad()).getRotations() - chassisAngularOffsetRadians);
  //     turningTalon.setControl(
  //       new PositionDutyCycle(
  //               Rotation2d.fromRadians(getEncoderRelativePositionRad()).getRotations())
  //           .withSlot(0));
  //   }
  // }

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
        new Rotation2d(Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble()).getRadians()));
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // return new SwerveModulePosition(
    //     drivingTalon.getPosition().getValue(),
    //     new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffsetRadians));
    return new SwerveModulePosition(
        drivingTalon.getPosition().getValueAsDouble(),
        new Rotation2d(Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble()).getRadians()));
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
            inputState, new Rotation2d(Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble()).getRadians()));

    // Ensure optimized state
    inputState.angle = Rotation2d.fromRadians(mod(inputState.angle.getRadians(), 2.0 * Math.PI));

    if (!overrideSlew) {
      inputState.speedMetersPerSecond = getDesiredVelocityMps(inputState.speedMetersPerSecond);
    }

    // Setting global desiredState to be optimized for the shuffleboard
    this.desiredState = inputState;

    desiredState.speedMetersPerSecond =
        this.throttleSpeed
            ? 0.8 * desiredState.speedMetersPerSecond
            : desiredState.speedMetersPerSecond;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingTalon.setControl(
        new VelocityDutyCycle(
                this.throttleSpeed
                    ? inputState.speedMetersPerSecond * 0.8
                    : inputState.speedMetersPerSecond)
            .withSlot(0));

    // i would definitely want this checked
    turningTalon.setControl(
        new PositionDutyCycle(
                inputState.angle.getRotations()) // 2024 code was in radians, but PositionDutyCycle seems to take in rotations
            .withSlot(0)); // TODO: check ... what does this mean?
  }

  /** eroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
    drivingTalon.setPosition(0.0);
  }

  // public double getEncoderAbsPositionRad() {
  //   return turningAbsoluteEncoder.getPosition();
  // }

  public double getEncoderRelativePositionRad() {
    return Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble()).getRadians();
  }

  public void throttleSpeed(boolean throttleSpeed) {
    this.throttleSpeed = throttleSpeed;
  }
}

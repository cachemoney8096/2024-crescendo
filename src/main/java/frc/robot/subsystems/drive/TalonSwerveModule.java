package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


public class TalonSwerveModule implements Sendable{
  public TalonFX drivingTalon;
  public TalonFX turningTalon;
  public CANcoder turningTalonAbsoluteEncoder;
  public double turningOffsetRotations;

  public SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public TalonSwerveModule(int drivingTalonCanId, int turningTalonCanId, int cancoderCanId, double turningOffsetRotationsValue){
    drivingTalon = new TalonFX(drivingTalonCanId);
    turningTalon = new TalonFX(turningTalonCanId);
    turningTalonAbsoluteEncoder = new CANcoder(cancoderCanId);
    turningOffsetRotations = turningOffsetRotationsValue;
    initAllDevices();
  }

  public void initAllDevices(){
    //TODO: All values subject to change

    //cancoder (turning talon encoder)
    CANcoderConfiguration ccToApply = new CANcoderConfiguration();
    ccToApply.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    ccToApply.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //TODO: subject to change
    ccToApply.MagnetSensor.MagnetOffset = turningOffsetRotations; //https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html
    turningTalonAbsoluteEncoder.getConfigurator().apply(ccToApply);

    //drive talon
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: subject to change
    toApply.Feedback.SensorToMechanismRatio =
        TalonDriveVars.SENSOR_TO_MECHANISM_RATIO_DRIVE;
    toApply.CurrentLimits.SupplyCurrentLimit =
        TalonDriveVars.DRIVING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.CurrentLimits.StatorCurrentLimit =
        TalonDriveVars.DRIVING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.Slot0.kP = TalonDriveVars.DRIVING_P;
    toApply.Slot0.kI = TalonDriveVars.DRIVING_I;
    toApply.Slot0.kD = TalonDriveVars.DRIVING_D;
    toApply.Slot0.kV = TalonDriveVars.DRIVING_FF;
    drivingTalon.getConfigurator().apply(toApply);
    final double fastUpdateFrequencyHz = 50.0; 
    final double slowUpdateFrequencyHz = 50.0;
    drivingTalon.getPosition().setUpdateFrequency(fastUpdateFrequencyHz);
    drivingTalon.getVelocity().setUpdateFrequency(fastUpdateFrequencyHz);
    drivingTalon.getClosedLoopProportionalOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopDerivativeOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopIntegratedOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopFeedForward().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.optimizeBusUtilization();
    drivingTalon.setPosition(0);

    //turning talon
    toApply.Feedback.SensorToMechanismRatio =
        TalonDriveVars.SENSOR_TO_MECHANISM_RATIO_TURNING;
    toApply.CurrentLimits.SupplyCurrentLimit =
        TalonDriveVars.TURNING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit =
        TalonDriveVars.TURNING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS;
    toApply.Slot0.kP = TalonDriveVars.TURNING_P;
    toApply.Slot0.kI = TalonDriveVars.TURNING_I;
    toApply.Slot0.kD = TalonDriveVars.TURNING_D;
    toApply.Slot0.kV = TalonDriveVars.TURNING_FF;
    toApply.Feedback.FeedbackRemoteSensorID = turningTalonAbsoluteEncoder.getDeviceID();
    toApply.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turningTalon.getConfigurator().apply(toApply);
    desiredState.angle = Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble());
  }

  /** Ensures the value a is in [0, b) */
  public static double mod(double a, double b) {
    double r = a % b;
    if (r < 0) {
      r += b;
    }
    return r;
  }

  /** Consider zeroing based on absolute values */
  public void considerZeroingEncoder(){
    if(Math.abs(turningTalonAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()) < 0.01){
      return;
    }
    if(Math.abs(turningTalonAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()-mod(turningTalon.getPosition().getValueAsDouble(), 1)) > TalonDriveVars.TURNING_ENCODER_ZERO_THRESHOLD_ROTATION){
      turningTalon.setPosition(turningTalonAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }
  }

  /** Get the current state of the module */
  public SwerveModuleState getState(){
    return new SwerveModuleState(
      drivingTalon.getVelocity().getValueAsDouble(),
      Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble())
    );
  }

  /** Get the current position of the module */
  public SwerveModuleState getPosition(){
    return new SwerveModuleState(
      drivingTalon.getPosition().getValueAsDouble(),
      Rotation2d.fromRotations(turningTalon.getPosition().getValueAsDouble())
    );
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

  /** Set the desired state for the module */
  public void setDesiredState(SwerveModuleState inputState, boolean overrideSlew){
    inputState =
        SwerveModuleState.optimize(
            inputState, Rotation2d.fromRotations(turningTalonAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()));
    inputState.angle = Rotation2d.fromRadians(mod(inputState.angle.getRadians(), 2.0 * Math.PI));
    if (!overrideSlew) {
      inputState.speedMetersPerSecond = getDesiredVelocityMps(inputState.speedMetersPerSecond);
    }
    this.desiredState = inputState;
    drivingTalon.setControl(
        new VelocityDutyCycle(inputState.speedMetersPerSecond).withSlot(0));
    turningTalon.setControl(new PositionVoltage(inputState.angle.getRotations()).withSlot(0));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
    drivingTalon.setPosition(0.0);
  }

  public void initSendable(SendableBuilder builder){
    
  }
}

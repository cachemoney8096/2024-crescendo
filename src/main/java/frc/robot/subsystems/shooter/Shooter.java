package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;
import java.util.Optional;

public class Shooter extends SubsystemBase {

  private final CANSparkMax motorA =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_A_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax motorB =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_B_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax pivotMotor =
      new CANSparkMax(RobotMap.SHOOTER_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final AbsoluteEncoder pivotMotorAbsoluteEncoder =
      pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private double pivotDesiredPositionDegrees = ShooterCal.STARTING_POSITION_DEGREES;

  private double prevVelocityRPM = 0;
  private Optional<Double> prevTimestamp = Optional.empty();

  /** double distance (meters), double angle (degrees) for pivotMotor */
  private InterpolatingDoubleTreeMap pivotAngleMap;

  private final SparkPIDController controllerA = motorA.getPIDController();
  private final SparkPIDController controllerB = motorB.getPIDController();
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(
          ShooterCal.PIVOT_MOTOR_kP,
          ShooterCal.PIVOT_MOTOR_kI,
          ShooterCal.PIVOT_MOTOR_kD,
          new TrapezoidProfile.Constraints(
              ShooterCal.PIVOT_MAX_VELOCITY_DEG_PER_SECOND,
              ShooterCal.PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED));

  public Shooter() {
    SparkMaxUtils.initWithRetry(this::initSparks, Constants.SPARK_INIT_RETRY_ATTEMPTS);
    pivotAngleMap = new InterpolatingDoubleTreeMap();
    pivotAngleMap.put(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);
    pivotAngleMap.put(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);
  }

  public void shoot() {
    controllerA.setReference(ShooterConstants.SHOOTER_MOTOR_SPEED_RPM, ControlType.kVelocity);
    controllerB.setReference(ShooterConstants.SHOOTER_MOTOR_SPEED_RPM, ControlType.kVelocity);
  }

  public void stop() {
    motorA.setVoltage(0);
    motorB.setVoltage(0);
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  public double getCosineArmAngle() {
    return Math.cos(getPivotPosition() - ShooterConstants.POSITION_WHEN_HORIZONTAL_DEGREES);
  }
  /**
   * Set arm to a specific angle by using distance (applies controlPosition())
   *
   * @param distance *
   */
  public void controlPositionWithDistance(double distance) {
    controlPosition(pivotAngleMap.get(distance));
  }
  /**
   * Set arm to a specific angle using voltage
   *
   * @param angle *
   */
  public void controlPosition(double angle) {
    pivotDesiredPositionDegrees = angle;
    pivotController.setGoal(angle);
    double timestamp = Timer.getFPGATimestamp();

    double armDemandVoltsA = pivotController.calculate(getPivotPosition());
    if (!prevTimestamp.isEmpty()) {
      armDemandVoltsA +=
          ShooterCal.PIVOT_MOTOR_FF.calculate(
              prevVelocityRPM,
              pivotController.getSetpoint().velocity,
              timestamp - prevTimestamp.get());
    } else {
      armDemandVoltsA +=
          ShooterCal.PIVOT_MOTOR_FF.calculate(pivotController.getSetpoint().velocity);
    }
    double armDemandVoltsB = ShooterCal.ARBITRARY_PIVOT_FEED_FORWARD_VOLTS * getCosineArmAngle();

    prevTimestamp = Optional.of(timestamp);
    prevVelocityRPM = pivotController.getSetpoint().velocity;

    pivotMotor.setVoltage(armDemandVoltsA + armDemandVoltsB);
  }

  public boolean atDesiredPosition() {
    return Math.abs(getPivotPosition() - pivotDesiredPositionDegrees)
        < ShooterConstants.PIVOT_ANGLE_MARGIN;
  }

  private double getPivotPosition() {
    return pivotMotorAbsoluteEncoder.getPosition() - ShooterConstants.PIVOT_ANGLE_OFFSET_DEGREES;
  }

  public boolean initSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(pivotMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorA.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorB.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(pivotMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(motorA.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(motorB.setIdleMode(IdleMode.kCoast));

    errors +=
        SparkMaxUtils.check(motorA.setSmartCurrentLimit(ShooterCal.SHOOTER_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(motorB.setSmartCurrentLimit(ShooterCal.SHOOTER_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(pivotMotor.setSmartCurrentLimit(ShooterCal.SHOOTER_CURRENT_LIMIT_AMPS));
    /** PID Stuff */
    errors += SparkMaxUtils.check(controllerA.setP(ShooterCal.MOTOR_A_kP));
    controllerA.setP(ShooterCal.MOTOR_A_kP);
    errors += SparkMaxUtils.check(controllerA.setI(ShooterCal.MOTOR_A_kI));
    controllerA.setI(ShooterCal.MOTOR_A_kI);
    errors += SparkMaxUtils.check(controllerA.setD(ShooterCal.MOTOR_A_kD));
    controllerA.setD(ShooterCal.MOTOR_A_kD);
    errors += SparkMaxUtils.check(controllerA.setFF(ShooterCal.MOTOR_A_kFF));
    controllerA.setFF(ShooterCal.MOTOR_A_kFF);

    errors += SparkMaxUtils.check(controllerB.setP(ShooterCal.MOTOR_A_kP));
    controllerB.setP(ShooterCal.MOTOR_A_kP);
    errors += SparkMaxUtils.check(controllerB.setI(ShooterCal.MOTOR_A_kI));
    controllerB.setI(ShooterCal.MOTOR_A_kI);
    errors += SparkMaxUtils.check(controllerB.setD(ShooterCal.MOTOR_A_kD));
    controllerB.setD(ShooterCal.MOTOR_A_kD);
    errors += SparkMaxUtils.check(controllerB.setFF(ShooterCal.MOTOR_A_kFF));
    controllerB.setFF(ShooterCal.MOTOR_A_kFF);
    return errors == 0;
  }
}

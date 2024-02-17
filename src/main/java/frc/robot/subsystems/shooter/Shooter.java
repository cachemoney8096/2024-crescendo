package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.SendableHelper;
import frc.robot.utils.SparkMaxUtils;
import java.util.Optional;

/** Shooter pivot (for shooting and for latching) and shooter wheels (for shooting). */
public class Shooter extends SubsystemBase {

  public enum ShooterMode {
    /** Not doing anything */
    IDLE,
    /** Spin up the shooter but stay at home position */
    SPIN_UP,
    /** Both spinning up the shooter and going to the right elevation angle */
    SHOOT,
    /** Holding pivot such that we can climb */
    PRELATCH,
    /** Holding pivot onto the chain */
    LATCH
  };

  private final CANSparkMax motorRight =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax motorLeftOne =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_ONE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax motorLeftTwo =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_TWO_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax pivotMotor =
      new CANSparkMax(RobotMap.SHOOTER_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  /** Configured to read degrees, zero is shooting straight down */
  private final AbsoluteEncoder pivotMotorAbsoluteEncoder =
      pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private double pivotDesiredPositionDegrees = ShooterCal.STARTING_POSITION_DEGREES;
  /** Not configured, defaults to Motor RPM. */
  private final RelativeEncoder motorRightRelEncoder = motorRight.getEncoder();
  /** Not configured, defaults to Motor RPM. */
  private final RelativeEncoder motorLeftOneRelEncoder = motorLeftOne.getEncoder();

  /** FPGA timestamp from previous cycle. Empty for first cycle only. */
  private Optional<Double> prevTimestamp = Optional.empty();

  /** Profiled velocity setpoint from previous cycle (degrees per second) */
  private double prevVelocityDegPerSec = 0;

  /** Map from goal distance (meters) to pivot angle (degrees) */
  private InterpolatingDoubleTreeMap pivotAngleMap;

  private final SparkPIDController controllerA = motorRight.getPIDController();
  private final SparkPIDController controllerB = motorLeftOne.getPIDController();
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(
          ShooterCal.PIVOT_MOTOR_kP,
          ShooterCal.PIVOT_MOTOR_kI,
          ShooterCal.PIVOT_MOTOR_kD,
          new TrapezoidProfile.Constraints(
              ShooterCal.PIVOT_MAX_VELOCITY_DEG_PER_SECOND,
              ShooterCal.PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED));

  /** What the shooter is currently doing */
  private ShooterMode shooterMode = ShooterMode.IDLE;

  /** How far are we away from the goal (in meters) */
  private double shooterDistanceMeters = 10.0;

  public Shooter() {
    pivotAngleMap = new InterpolatingDoubleTreeMap();
    pivotAngleMap.put(0.0, 114.0);
    pivotAngleMap.put(100.0, 114.0);

    SparkMaxUtils.initWithRetry(this::initSparks, Constants.SPARK_INIT_RETRY_ATTEMPTS);
  }

  public boolean initSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(pivotMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorRight.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorLeftOne.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorLeftTwo.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(motorLeftTwo.follow(motorLeftOne));

    errors += SparkMaxUtils.check(pivotMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(motorRight.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(motorLeftOne.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(motorLeftTwo.setIdleMode(IdleMode.kCoast));

    errors +=
        SparkMaxUtils.check(pivotMotor.setSmartCurrentLimit(ShooterCal.SHOOTER_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(motorRight.setSmartCurrentLimit(ShooterCal.SHOOTER_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            motorLeftOne.setSmartCurrentLimit(ShooterCal.SHOOTER_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            motorLeftTwo.setSmartCurrentLimit(ShooterCal.SHOOTER_CURRENT_LIMIT_AMPS));

    // PID Stuff
    errors += SparkMaxUtils.check(controllerA.setP(ShooterCal.MOTOR_A_kP));
    errors += SparkMaxUtils.check(controllerA.setI(ShooterCal.MOTOR_A_kI));
    errors += SparkMaxUtils.check(controllerA.setD(ShooterCal.MOTOR_A_kD));
    errors += SparkMaxUtils.check(controllerA.setFF(ShooterCal.MOTOR_A_kFF));

    errors += SparkMaxUtils.check(controllerB.setP(ShooterCal.MOTOR_B_kP));
    errors += SparkMaxUtils.check(controllerB.setI(ShooterCal.MOTOR_B_kI));
    errors += SparkMaxUtils.check(controllerB.setD(ShooterCal.MOTOR_B_kD));
    errors += SparkMaxUtils.check(controllerB.setFF(ShooterCal.MOTOR_B_kFF));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                pivotMotorAbsoluteEncoder, ShooterConstants.ABS_ENCODER_GEAR_RATIO));

    return errors == 0;
  }

  public void burnFlashSparks() {
    Timer.delay(0.005);
    pivotMotor.burnFlash();
    Timer.delay(0.005);
    motorLeftOne.burnFlash();
    Timer.delay(0.005);
    motorLeftTwo.burnFlash();
    Timer.delay(0.005);
    motorRight.burnFlash();
  }

  /** Set the shooter to a new mode. If shoot, make sure to also call setShooterDistance . */
  public void setShooterMode(ShooterMode newMode) {
    if (shooterMode != newMode) {
      pivotController.reset(getPivotPosition());
    }
    shooterMode = newMode;
  }

  public void setShooterDistance(double newDistanceMeters) {
    shooterDistanceMeters = newDistanceMeters;
  }

  public boolean atDesiredPosition() {
    return Math.abs(getPivotPosition() - pivotDesiredPositionDegrees)
        < ShooterCal.PIVOT_ANGLE_MARGIN_DEG;
  }

  public boolean clearOfConveyorZone() {
    return getPivotPosition() < ShooterCal.CONVEYOR_ZONE_THRESHOLD_DEGREES;
  }

  public boolean isShooterSpunUp() {
    final boolean motorRightSpunUp =
        Math.abs(motorRightRelEncoder.getVelocity() - ShooterCal.SHOOTER_MOTOR_SPEED_RPM)
            < ShooterCal.SHOOTER_SPEED_MARGIN_RPM;
    final boolean motorLeftOneSpunUp =
        Math.abs(motorLeftOneRelEncoder.getVelocity() - ShooterCal.SHOOTER_MOTOR_SPEED_RPM)
            < ShooterCal.SHOOTER_SPEED_MARGIN_RPM;
    return motorRightSpunUp && motorLeftOneSpunUp;
  }

  private void spinUpShooter() {
    controllerA.setReference(ShooterCal.SHOOTER_MOTOR_SPEED_RPM, ControlType.kVelocity);
    controllerB.setReference(ShooterCal.SHOOTER_MOTOR_SPEED_RPM, ControlType.kVelocity);
  }

  private void stopShooter() {
    motorRight.setVoltage(0);
    motorLeftOne.setVoltage(0);
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  private double getCosineArmAngle() {
    return Math.cos(getPivotPosition() - ShooterConstants.POSITION_WHEN_HORIZONTAL_DEGREES);
  }

  /**
   * Sends pivot to the prelatch position (applies controlPosition()). Should be called every cycle
   * when this is desired.
   */
  private void controlPositionToPreLatch() {
    controlPosition(ShooterCal.PRE_LATCH_ANGLE_DEGREES);
  }
  /**
   * Sends pivot to the latch position (applies controlPosition()). Should be called every cycle
   * when this is desired.
   */
  private void controlPositionToLatch() {
    controlPosition(ShooterCal.LATCH_ANGLE_DEGREES);
  }

  /**
   * Set arm to a specific angle by using distance (applies controlPosition()) Should be called
   * every cycle when this is desired.
   *
   * @param distance *
   */
  private void controlPositionWithDistance(double distance) {
    controlPosition(pivotAngleMap.get(distance));
  }

  /**
   * Set pivot to a specific angle using voltage. Needs to be called every cycle.
   *
   * @param angleDeg Desired pivot position. *
   */
  private void controlPosition(double angleDeg) {
    pivotDesiredPositionDegrees = angleDeg;
    pivotController.setGoal(angleDeg);
    final double timestamp = Timer.getFPGATimestamp();

    final double armDemandVoltsA = pivotController.calculate(getPivotPosition());
    double armDemandVoltsB;
    if (!prevTimestamp.isEmpty()) {
      armDemandVoltsB =
          ShooterCal.PIVOT_MOTOR_FF.calculate(
              prevVelocityDegPerSec,
              pivotController.getSetpoint().velocity,
              timestamp - prevTimestamp.get());
    } else {
      armDemandVoltsB = ShooterCal.PIVOT_MOTOR_FF.calculate(pivotController.getSetpoint().velocity);
    }
    final double armDemandVoltsC =
        ShooterCal.ARBITRARY_PIVOT_FEED_FORWARD_VOLTS * getCosineArmAngle();

    pivotMotor.setVoltage(armDemandVoltsA + armDemandVoltsB + armDemandVoltsC);

    prevTimestamp = Optional.of(timestamp);
    prevVelocityDegPerSec = pivotController.getSetpoint().velocity;
  }

  private double getPivotPosition() {
    return pivotMotorAbsoluteEncoder.getPosition() - ShooterCal.PIVOT_ANGLE_OFFSET_DEGREES;
  }

  @Override
  public void periodic() {
    switch (shooterMode) {
      case IDLE:
        stopShooter();
        controlPosition(ShooterCal.STARTING_POSITION_DEGREES);
        break;
      case SPIN_UP:
        spinUpShooter();
        controlPosition(ShooterCal.STARTING_POSITION_DEGREES);
        break;
      case SHOOT:
        spinUpShooter();
        controlPositionWithDistance(shooterDistanceMeters);
        break;
      case LATCH:
        stopShooter();
        controlPositionToLatch();
        break;
      case PRELATCH:
        stopShooter();
        controlPositionToPreLatch();
        break;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SendableHelper.addChild(builder, this, pivotController, "PivotController");
    builder.addDoubleProperty("Pivot desired pos (deg)", () -> pivotDesiredPositionDegrees, null);
    builder.addDoubleProperty("abs enc pos (deg)", pivotMotorAbsoluteEncoder::getPosition, null);
    builder.addDoubleProperty("pivot pos (deg)", this::getPivotPosition, null);
    builder.addDoubleProperty(
        "pivot velocity (deg/sec)", pivotMotorAbsoluteEncoder::getVelocity, null);
    builder.addBooleanProperty("At desired pos", this::atDesiredPosition, null);
    builder.addDoubleProperty(
        "Motor Right velocity (in per sec)", motorRightRelEncoder::getVelocity, null);
    builder.addDoubleProperty(
        "Motor Left One velocity (in per sec)", motorLeftOneRelEncoder::getVelocity, null);
    builder.addBooleanProperty("At desired shooter speeds", this::isShooterSpunUp, null);
    builder.addStringProperty("Current Mode", () -> shooterMode.toString(), null);
    builder.addDoubleProperty("Shooter Dist (m)", () -> shooterDistanceMeters, null);
  }
}

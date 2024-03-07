package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    LATCH,
    /** Shooter set high enough to shoot note over stage */
    SHOOT_CLEAR_STAGE
  };

  private final CANSparkMax motorRight =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax motorLeftOne =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_ONE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax motorLeftTwo =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_TWO_CAN_ID, MotorType.kBrushless);
  public final CANSparkMax pivotMotor =
      new CANSparkMax(RobotMap.SHOOTER_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  /** Configured to read degrees, zero is shooting straight down */
  private final AbsoluteEncoder pivotMotorAbsoluteEncoder =
      pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  /** Configured to read degrees, zero is shooting straight down */
  private final RelativeEncoder pivotMotorEncoder = pivotMotor.getEncoder();

  private double pivotDesiredPositionDegrees = ShooterCal.STARTING_POSITION_DEGREES;
  /** Not configured, defaults to Motor RPM. */
  private final RelativeEncoder motorRightRelEncoder = motorRight.getEncoder();
  /** Not configured, defaults to Motor RPM. */
  private final RelativeEncoder motorLeftOneRelEncoder = motorLeftOne.getEncoder();

  private final RelativeEncoder motorLeftTwoRelEncoder = motorLeftTwo.getEncoder();

  /** FPGA timestamp from previous cycle. Empty for first cycle only. */
  private Optional<Double> prevTimestamp = Optional.empty();

  /** Profiled velocity setpoint from previous cycle (degrees per second) */
  private double prevDesiredVelocityDegPerSec = 0;

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

  private boolean allowShooterMovement = false;

  /** How far are we away from the goal (in meters) */
  private double shooterDistanceMeters = 10.0;

  // Stuff for debug
  double desiredAccelDegPerSecSq = 0.0;
  double actualAccelDegPerSecSq = 0.0;
  double desiredVelDegPerSec = 0.0;
  double actualVelDegPerSec = 0.0;
  double prevActualVelocitDegPerSec = 0.0;
  private double periodSec = 0.0;
  private double armDemandVoltsA;
  private double armDemandVoltsB;
  private double armDemandVoltsC;
  private double armDemandVoltsD;

  /** Set to true once speaker prep finished (we are rotated and distanced to a tag) */
  public boolean readyToShoot = false;

  public Shooter() {
    pivotAngleMap = new InterpolatingDoubleTreeMap();
    pivotAngleMap.put(1.16, 144.0);
    pivotAngleMap.put(2.77, 122.0);
    pivotAngleMap.put(4.66, 112.0);

    SparkMaxUtils.initWithRetry(this::initSparks, Constants.SPARK_INIT_RETRY_ATTEMPTS);

    pivotController.reset(getPivotPositionDegrees());
    pivotMotorEncoder.setPosition(getPivotPositionFromAbs());
  }

  public boolean initSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(pivotMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorRight.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorLeftOne.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(motorLeftTwo.restoreFactoryDefaults());

    Timer.delay(0.1);

    motorLeftOne.setInverted(true);
    errors += SparkMaxUtils.check(motorLeftTwo.follow(motorLeftOne));

    errors += SparkMaxUtils.check(pivotMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(motorRight.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(motorLeftOne.setIdleMode(IdleMode.kCoast));
    errors += SparkMaxUtils.check(motorLeftTwo.setIdleMode(IdleMode.kCoast));

    errors +=
        SparkMaxUtils.check(pivotMotor.setSmartCurrentLimit(ShooterCal.PIVOT_CURRENT_LIMIT_AMPS));
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
                pivotMotorEncoder, ShooterConstants.PIVOT_MOTOR_GEAR_RATIO));

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
    shooterMode = newMode;
    pivotController.reset(getPivotPositionDegrees());
    this.allowShooterMovement = true;
  }

  public void dontAllowShooterMovement() {
    this.allowShooterMovement = false;
    pivotMotor.setVoltage(0.0);
    motorRight.setVoltage(0.0);
    motorLeftOne.setVoltage(0.0);
  }

  public void setShooterDistance(double newDistanceMeters) {
    shooterDistanceMeters = newDistanceMeters;
  }

  public boolean atDesiredPosition() {
    return Math.abs(getPivotPositionDegrees() - pivotDesiredPositionDegrees)
        < ShooterCal.PIVOT_ANGLE_MARGIN_DEG;
  }

  public boolean clearOfConveyorZone() {
    return getPivotPositionDegrees() < ShooterCal.CONVEYOR_ZONE_THRESHOLD_DEGREES;
  }

  public boolean isShooterSpunUp() {
    final double leftSpeedRpm = motorLeftOneRelEncoder.getVelocity();
    final double rightSpeedRpm = motorRightRelEncoder.getVelocity();
    final double thresholdRpm = 2000.0;
    return leftSpeedRpm > thresholdRpm && rightSpeedRpm > thresholdRpm;
  }

  public void spinUpShooter() {
    motorRight.setVoltage(7.0);
    motorLeftOne.setVoltage(7.0);
  }

  public void spinUpShooter(double voltage) {
    motorRight.setVoltage(voltage);
    motorLeftOne.setVoltage(voltage);
  }

  private void stopShooter() {
    motorRight.setVoltage(0);
    motorLeftOne.setVoltage(0);
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  private double getCosineArmAngle() {
    return Math.cos(
        Units.degreesToRadians(
            getPivotPositionDegrees() - ShooterConstants.POSITION_WHEN_HORIZONTAL_DEGREES));
  }

  /**
   * Sends pivot to the prelatch position (applies controlPosition()). Should be called every cycle
   * when this is desired.
   */
  private void controlPositionToPreLatch() {
    controlPosition(ShooterCal.PRE_LATCH_ANGLE_DEGREES, false);
  }

  /**
   * Sends pivot to the latch position (applies controlPosition()). Should be called every cycle
   * when this is desired.
   */
  private void controlPositionToLatch() {
    controlPosition(ShooterCal.LATCH_ANGLE_DEGREES, false);
  }

  /**
   * Sends pivot to the latch position (applies controlPosition()). Should be called every cycle
   * when this is desired.
   */
  private void controlPositionToHoldLatch() {
    controlPosition(ShooterCal.LATCH_ANGLE_DEGREES, true);
  }

  /**
   * Set arm to a specific angle by using distance (applies controlPosition()) Should be called
   * every cycle when this is desired.
   *
   * @param distance *
   */
  private void controlPositionWithDistance(double distance) {
    controlPosition(pivotAngleMap.get(distance), false);
  }

  /**
   * Set pivot to a specific angle using voltage. Needs to be called every cycle.
   *
   * @param angleDeg Desired pivot position. *
   */
  private void controlPosition(double angleDeg, boolean holdLatchVoltage) {
    pivotDesiredPositionDegrees = angleDeg;
    pivotController.setGoal(angleDeg);
    final double timestamp = Timer.getFPGATimestamp();
    actualVelDegPerSec = pivotMotorAbsoluteEncoder.getVelocity();

    armDemandVoltsA = pivotController.calculate(getPivotPositionDegrees());
    if (!prevTimestamp.isEmpty()) {
      armDemandVoltsB =
          ShooterCal.PIVOT_MOTOR_FF.calculate(
              prevDesiredVelocityDegPerSec,
              pivotController.getSetpoint().velocity,
              timestamp - prevTimestamp.get());
      desiredAccelDegPerSecSq =
          (pivotController.getSetpoint().velocity - prevDesiredVelocityDegPerSec)
              / (timestamp - prevTimestamp.get());
      actualAccelDegPerSecSq =
          (actualVelDegPerSec - prevActualVelocitDegPerSec) / (timestamp - prevTimestamp.get());
      if (Math.abs(desiredAccelDegPerSecSq) > 50.0) {
        desiredAccelDegPerSecSq = 0.0;
      }
      periodSec = timestamp - prevTimestamp.get();
    } else {
      armDemandVoltsB = ShooterCal.PIVOT_MOTOR_FF.calculate(pivotController.getSetpoint().velocity);
    }
    armDemandVoltsC = ShooterCal.ARBITRARY_PIVOT_FEED_FORWARD_VOLTS * getCosineArmAngle();

    armDemandVoltsD =
        Math.signum(pivotController.getPositionError())
            * ShooterCal.ARBITRARY_PIVOT_FEED_FORWARD_VOLTS_KS;

    pivotMotor.setVoltage(
        holdLatchVoltage
            ? ShooterCal.HOLD_LATCH_ARBITRARY_ADDITION_VOLTS
            : (armDemandVoltsA + armDemandVoltsB + armDemandVoltsC + armDemandVoltsD));

    desiredVelDegPerSec = pivotController.getSetpoint().velocity;

    prevActualVelocitDegPerSec = actualVelDegPerSec;
    prevTimestamp = Optional.of(timestamp);
    prevDesiredVelocityDegPerSec = pivotController.getSetpoint().velocity;
  }

  private double getPivotPositionFromAbs() {
    return pivotMotorAbsoluteEncoder.getPosition() - ShooterCal.PIVOT_ANGLE_OFFSET_DEGREES;
  }

  private double getPivotPositionDegrees() {
    return pivotMotorEncoder.getPosition();
  }

  public void considerZeroingEncoder() {
    if (Math.abs(getPivotPositionDegrees() - getPivotPositionFromAbs())
        > ShooterCal.PIVOT_ENCODER_ZEROING_THRESHOLD_DEG) {
      pivotMotorEncoder.setPosition(getPivotPositionFromAbs());
      pivotController.reset(pivotMotorEncoder.getPosition());
    }
  }

  private boolean closeToLatch() {
    return shooterMode == ShooterMode.LATCH
        && getPivotPositionDegrees() > ShooterCal.LATCH_ANGLE_DEGREES - 2.0;
  }

  @Override
  public void periodic() {
    if (allowShooterMovement) {
      switch (shooterMode) {
        case IDLE:
          stopShooter();
          controlPosition(ShooterCal.STARTING_POSITION_DEGREES, false);
          break;
        case SPIN_UP:
          spinUpShooter();
          controlPosition(ShooterCal.STARTING_POSITION_DEGREES, false);
          break;
        case SHOOT:
          spinUpShooter();
          controlPositionWithDistance(shooterDistanceMeters);
          break;
        case SHOOT_CLEAR_STAGE:
          readyToShoot = true;
          spinUpShooter(ShooterCal.SHOOT_CLEAR_STAGE_VOLTAGE);
          controlPosition(ShooterCal.SHOOT_CLEAR_STAGE_ANGLE_DEGREES, false);
          break;
        case LATCH:
          stopShooter();
          if (closeToLatch()) {
            controlPositionToHoldLatch();
          } else {
            controlPositionToLatch();
          }
          break;
        case PRELATCH:
          stopShooter();
          controlPositionToPreLatch();
          break;
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SendableHelper.addChild(builder, this, pivotController, "PivotController");
    builder.addDoubleProperty("Pivot desired pos (deg)", () -> pivotDesiredPositionDegrees, null);
    builder.addDoubleProperty("abs enc pos (deg)", pivotMotorAbsoluteEncoder::getPosition, null);
    builder.addDoubleProperty("pivot pos (deg)", this::getPivotPositionDegrees, null);
    builder.addDoubleProperty("pivot pos from abs (deg)", this::getPivotPositionFromAbs, null);
    builder.addDoubleProperty(
        "pivot velocity (deg/sec)", pivotMotorAbsoluteEncoder::getVelocity, null);
    builder.addBooleanProperty("At desired pos", this::atDesiredPosition, null);
    builder.addDoubleProperty(
        "Motor Right velocity (rpm)", motorRightRelEncoder::getVelocity, null);
    builder.addDoubleProperty(
        "Motor Left One velocity (rpm)", motorLeftOneRelEncoder::getVelocity, null);
    builder.addDoubleProperty(
        "Motor Left Two velocity (rpm)", motorLeftTwoRelEncoder::getVelocity, null);
    // builder.addDoubleProperty("", controllerA.get)
    builder.addBooleanProperty("At desired shooter speeds", this::isShooterSpunUp, null);
    builder.addStringProperty("Current Mode", () -> shooterMode.toString(), null);
    builder.addDoubleProperty("Shooter Dist (m)", () -> shooterDistanceMeters, null);
    builder.addDoubleProperty("PID (V)", () -> armDemandVoltsA, null);
    builder.addDoubleProperty("Feedforward (V)", () -> armDemandVoltsB, null);
    builder.addDoubleProperty("Gravity Comp (V)", () -> armDemandVoltsC, null);
    builder.addDoubleProperty("PID KS (V)", () -> armDemandVoltsD, null);
    builder.addDoubleProperty("Goal (deg)", () -> pivotController.getGoal().position, null);
    builder.addDoubleProperty(
        "Setpoint position (deg)", () -> pivotController.getSetpoint().position, null);
    builder.addDoubleProperty(
        "Setpoint Velocity (deg per s)", () -> pivotController.getSetpoint().velocity, null);
    builder.addDoubleProperty("Actual Velocity (deg per s)", () -> actualVelDegPerSec, null);
    builder.addDoubleProperty("Desired Velocity (deg per s)", () -> desiredVelDegPerSec, null);
    builder.addDoubleProperty("Actual Accel (deg per s2)", () -> actualAccelDegPerSecSq, null);
    builder.addDoubleProperty("Desired Accel (deg per s2)", () -> desiredAccelDegPerSecSq, null);
    builder.addDoubleProperty(
        "Position Error (deg)", () -> pivotController.getPositionError(), null);
    builder.addDoubleProperty("Period (sec)", () -> periodSec, null);
    builder.addDoubleProperty(
        "Abs to Rel Diff (deg)",
        () -> {
          return getPivotPositionDegrees() - getPivotPositionFromAbs();
        },
        null);
    builder.addBooleanProperty("Clear of conveyor", this::clearOfConveyorZone, null);
    builder.addBooleanProperty("Close to latch", this::closeToLatch, null);
    builder.addBooleanProperty("Ready to shoot", () -> readyToShoot, null);
  }
}

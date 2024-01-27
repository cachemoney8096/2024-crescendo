package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter {

  private final CANSparkMax motorA =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_A_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder motorAEncoder = motorA.getEncoder();
  private final CANSparkMax motorB =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_B_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder motorBEncoder = motorB.getEncoder();
  private final CANSparkMax pivotMotor =
      new CANSparkMax(RobotMap.SHOOTER_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final AbsoluteEncoder shooterAbsoluteEncoder =
      pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private double pivotDesiredPosition = ShooterCal.STARTING_POSITION_DEGREES;
  private final int SMART_MOTION_SLOT = 0;

  /** Double distance, Double angle for pivotMotor */
  private InterpolatingDoubleTreeMap pivotAngleMap;

  private final SparkPIDController controllerA;
  private final SparkPIDController controllerB;
  private final SparkPIDController pivotController;

  public Shooter() {
    pivotAngleMap = new InterpolatingDoubleTreeMap();
    pivotAngleMap.put(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);
    pivotAngleMap.put(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

    controllerA = motorA.getPIDController();
    controllerB = motorB.getPIDController();
    pivotController = pivotMotor.getPIDController();

    controllerA.setP(ShooterCal.MOTOR_A_kP);
    controllerA.setI(ShooterCal.MOTOR_A_kI);
    controllerA.setD(ShooterCal.MOTOR_A_kD);
    controllerA.setFF(ShooterCal.MOTOR_A_kFF);

    controllerB.setP(ShooterCal.MOTOR_B_kP);
    controllerB.setI(ShooterCal.MOTOR_B_kI);
    controllerB.setD(ShooterCal.MOTOR_B_kD);
    controllerB.setFF(ShooterCal.MOTOR_B_kFF);

    pivotController.setP(ShooterCal.PIVOT_MOTOR_kP);
    pivotController.setI(ShooterCal.PIVOT_MOTOR_kI);
    pivotController.setD(ShooterCal.PIVOT_MOTOR_kD);
    pivotController.setFF(ShooterCal.PIVOT_MOTOR_kFF);

  }

  public void shoot(double speedA, double speedB) {
    controllerA.setReference(speedA, ControlType.kVelocity);
    controllerB.setReference(speedB, ControlType.kVelocity);
  }

  public void stop() {
    motorA.stopMotor();
    motorB.stopMotor();
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  public double getCosineIntakeAngle() {
    return Math.cos(
        shooterAbsoluteEncoder.getPosition() - ShooterConstants.POSITION_WHEN_HORIZONTAL_DEGREES);
  }

  public void setPivotAngle(double distance) {
    pivotMotor.set(pivotAngleMap.get(distance));
  }
  /** NOT DONE */
  public void setLatchAngle() {
    pivotMotor.set(ShooterConstants.LATCH_ANGLE_DEGREES);
  }

   public void correctPosition() {
    pivotController.setReference(
        ShooterCal.STARTING_POSITION_DEGREES, 
        CANSparkMax.ControlType.kSmartMotion,
        SMART_MOTION_SLOT, // check this variable
        ShooterCal.ARBITRARY_FEED_FORWARD_VOLTS * getCosineIntakeAngle(),
        ArbFFUnits.kVoltage);
    pivotDesiredPosition = ShooterCal.STARTING_POSITION_DEGREES;
  }
}

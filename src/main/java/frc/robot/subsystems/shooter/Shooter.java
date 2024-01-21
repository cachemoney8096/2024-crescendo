package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
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

  /** Double distance, Double angle for pivotMotor */
  private InterpolatingDoubleTreeMap pivotAngleMap;

  private final SparkPIDController controllerA;
  private final SparkPIDController controllerB;

  public Shooter() {
    pivotAngleMap = new InterpolatingDoubleTreeMap();
    pivotAngleMap.put(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);
    pivotAngleMap.put(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

    controllerA = motorA.getPIDController();
    controllerB = motorB.getPIDController();

    controllerA.setP(ShooterCal.MOTOR_A_kP);
    controllerA.setI(ShooterCal.MOTOR_A_kI);
    controllerA.setD(ShooterCal.MOTOR_A_kD);
    controllerA.setFF(ShooterCal.MOTOR_A_kFF);

    controllerB.setP(ShooterCal.MOTOR_B_kP);
    controllerB.setI(ShooterCal.MOTOR_B_kI);
    controllerB.setD(ShooterCal.MOTOR_B_kD);
    controllerB.setFF(ShooterCal.MOTOR_B_kFF);

  }

  public void shoot(double speedA, double speedB) {
    controllerA.setReference(speedA, ControlType.kVelocity);
    controllerB.setReference(speedB, ControlType.kVelocity);
  }

  public void stop() {
    motorA.stopMotor();
    motorB.stopMotor();
  }

  public void setPivotAngle(double distance) {
    pivotMotor.set(pivotAngleMap.get(distance));
  }

  public void setLatchAngle(double position) {
    pivotMotor.set(position);
  }
}

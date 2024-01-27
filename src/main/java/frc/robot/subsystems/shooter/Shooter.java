package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase{

  private final CANSparkMax motorA =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_A_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax motorB =
      new CANSparkMax(RobotMap.SHOOTER_MOTOR_B_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax pivotMotor =
      new CANSparkMax(RobotMap.SHOOTER_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  private final AbsoluteEncoder pivotMotorAbsoluteEncoder =
      pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private double pivotDesiredPosition = ShooterCal.STARTING_POSITION_DEGREES;

  /** Double distance, Double angle for pivotMotor */
  private InterpolatingDoubleTreeMap pivotAngleMap;

  private final SparkPIDController controllerA;
  private final SparkPIDController controllerB;
  private final ProfiledPIDController pivotController = new ProfiledPIDController(ShooterCal.PIVOT_MOTOR_kP, ShooterCal.PIVOT_MOTOR_kI, ShooterCal.PIVOT_MOTOR_kD, new TrapezoidProfile.Constraints(ShooterCal.PIVOT_MAX_VELOCITY_DEG_PER_SECOND, ShooterCal.PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED));

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
    controllerA.setReference(0, ControlType.kVelocity);
    controllerB.setReference(0, ControlType.kVelocity);
  }

  /** Returns the cosine of the intake angle in degrees off of the horizontal. */
  public double getCosineArmAngle() {
    return Math.cos(
        pivotMotorAbsoluteEncoder.getPosition() - ShooterConstants.POSITION_WHEN_HORIZONTAL_DEGREES);
  }

  private void controlPosition(double distance){
    pivotController.setGoal(pivotAngleMap.get(distance));
    double armDemandVoltsA = pivotController.calculate(pivotMotorAbsoluteEncoder.getPosition());
    double armDemandVoltsB =
        ShooterCal.PIVOT_MOTOR_FF.calculate(pivotController.getSetpoint().velocity);
    double armDemandVoltsC = ShooterCal.ARBITRARY_PIVOT_FEED_FORWARD_VOLTS * getCosineArmAngle();
    pivotMotor.setVoltage(armDemandVoltsA + armDemandVoltsB + armDemandVoltsC);
  }

//**Shooter latches (post climbing) **/
  public void latch(){
    controlPosition(Constants.PLACEHOLDER_DOUBLE);
  }

  //** Puts shooter in position to latch (before climbing)**/
  public void setLatchPosition() {
    controlPosition(Constants.PLACEHOLDER_DOUBLE);
  }
//**Checks to see if shooter is in position to latch **/
  public boolean checkLatch() {
    if (pivotDesiredPosition == ShooterConstants.LATCH_ANGLE_DEGREES){
      return true;
    } else {
      return false;
    }
  }

}

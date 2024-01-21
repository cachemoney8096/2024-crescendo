package frc.robot.subsystems.conveyer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;

public class Conveyer extends SubsystemBase {
  private CANSparkMax frontMotor =
      new CANSparkMax(RobotMap.FRONT_CONVEYER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax backMotor =
      new CANSparkMax(RobotMap.BACK_CONVEYER_CAN_ID, MotorType.kBrushless);
  

  public Conveyer() {
    SparkMaxUtils.initWithRetry(this::setUpConveyerSparks, ConveyerCal.SPARK_INIT_RETRY_ATTEMPTS);
  }

  /** Does all the initialization for the sparks, return true on success */
  private boolean setUpConveyerSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(frontMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(backMotor.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(frontMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(backMotor.setIdleMode(IdleMode.kBrake));

    frontMotor.setInverted(ConveyerConstants.FRONT_MOTOR_INVERTED);
    backMotor.setInverted(ConveyerConstants.BACK_MOTOR_INVERTED);

    errors +=
        SparkMaxUtils.check(
            frontMotor.setSmartCurrentLimit(ConveyerCal.CONVEYER_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            backMotor.setSmartCurrentLimit(ConveyerCal.CONVEYER_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  public void prepareToShoot() {
    frontMotor.set(ConveyerCal.PREPARE_TO_SHOOT_FRONT_SPEED);
    backMotor.set(ConveyerCal.PREPARE_TO_SHOOT_BACK_SPEED);
  }

  public void prepareTrapOrAmp() {
    frontMotor.set(ConveyerCal.PREPARE_TO_AMP_TRAP_FRONT_SPEED);
    backMotor.set(ConveyerCal.PREPARE_TO_AMP_TRAP_BACK_SPEED);
  }

  public void hold() {
    frontMotor.set(ConveyerCal.FRONT_HOLD_SPEED);
    backMotor.set(ConveyerCal.BACK_HOLD_SPEED);
  }

  public void stop() {
    frontMotor.set(0.0);
    backMotor.set(0.0);
  }
}

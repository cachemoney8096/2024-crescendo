package frc.robot.subsystems.elevator;

import java.util.TreeMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.SparkMaxUtils;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition{
    HOME,
    SCORE_TRAP,
    SCORE_AMP,
    PRE_CLIMB,
    CLIMB,
    POST_CLIMB
  }

  public CANSparkMax leftMotor = new CANSparkMax(RobotMap.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
  public CANSparkMax rightMotor = new CANSparkMax(RobotMap.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  private final AbsoluteEncoder leftMotorEncoderAbs = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();
  private final AbsoluteEncoder rightMotorEncoderAbs = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private ProfiledPIDController noteScoringElevatorController = new ProfiledPIDController(ElevatorCal.NOTE_SCORING_P,
      ElevatorCal.NOTE_SCORING_I, ElevatorCal.NOTE_SCORING_D, new TrapezoidProfile.Constraints(
          ElevatorCal.MAX_VELOCITY_PER_SECOND,
          ElevatorCal.MAX_ACCELERATION_PER_SECOND_SQUARED));
  private ProfiledPIDController chainGrabberElevatorController = new ProfiledPIDController(ElevatorCal.GRABBING_P,
      ElevatorCal.GRABBING_I, ElevatorCal.GRABBING_D, new TrapezoidProfile.Constraints(
          ElevatorCal.MAX_VELOCITY_PER_SECOND,
          ElevatorCal.MAX_ACCELERATION_PER_SECOND_SQUARED));

  private ProfiledPIDController currentPIDController = noteScoringElevatorController;

  private TreeMap<ElevatorPosition, Double> elevatorPositions;

  private ElevatorPosition desiredPosition;

  private boolean usingScoringPID = true;

  public Elevator() {
    SparkMaxUtils.initWithRetry(this::initSparks, ElevatorCal.MAX_INIT_RETRY_ATTEMPTS);
    elevatorPositions = new TreeMap<ElevatorPosition, Double>();
    elevatorPositions.put(ElevatorPosition.HOME, ElevatorCal.POSITION_HOME);
    elevatorPositions.put(ElevatorPosition.SCORE_AMP, ElevatorCal.POSITION_SCORE_AMP);
    elevatorPositions.put(ElevatorPosition.SCORE_TRAP, ElevatorCal.POSITION_SCORE_TRAP);
    elevatorPositions.put(ElevatorPosition.PRE_CLIMB, ElevatorCal.POSITION_PRE_CLIMB);
    elevatorPositions.put(ElevatorPosition.CLIMB, ElevatorCal.POSITION_CLIMB);
    elevatorPositions.put(ElevatorPosition.POST_CLIMB, ElevatorCal.POSITION_POST_CLIMB);
  }

  public void useScoringPID(boolean b){
    usingScoringPID = b;
    if(b){
      currentPIDController = noteScoringElevatorController;
    }
    else{
      currentPIDController = chainGrabberElevatorController;
    }
  }

  private void controlPosition(ElevatorPosition pos){
    currentPIDController.setGoal(elevatorPositions.get(pos));
    double elevatorDemandVolts = currentPIDController.calculate(leftMotorEncoder.getPosition());
    if(usingScoringPID){
      elevatorDemandVolts +=
        ElevatorCal.NOTE_SCORING_FF.calculate(currentPIDController.getSetpoint().velocity);
    }
    else{
      elevatorDemandVolts +=
        ElevatorCal.GRABBING_FF.calculate(currentPIDController.getSetpoint().velocity);
    }
    leftMotor.setVoltage(elevatorDemandVolts);
  }

  public void setDesiredPosition(ElevatorPosition desiredPosition){
    this.desiredPosition = desiredPosition;
  }

  @Override
  public void periodic(){
    controlPosition(desiredPosition);
  }

  private boolean initSparks() {
    int errors = 0;
    errors += SparkMaxUtils.check(leftMotor.restoreFactoryDefaults());
    errors += SparkMaxUtils.check(rightMotor.restoreFactoryDefaults());

    errors += SparkMaxUtils.check(rightMotor.follow(leftMotor, true));

    errors += SparkMaxUtils.check(leftMotorEncoderAbs.setInverted(false));

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(leftMotorEncoderAbs, 1.0));
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setDegreesFromGearRatio(
                rightMotorEncoderAbs, ElevatorCal.ELEVATOR_RIGHT_ABSOLUTE_ENCODER_RATIO));
    errors +=
        SparkMaxUtils.check(
            leftMotorEncoder.setPositionConversionFactor(
                ElevatorCal.ELEVATOR_MOTOR_ENCODER_IN_PER_REV));
    errors +=
        SparkMaxUtils.check(
            leftMotorEncoder.setVelocityConversionFactor(
                ElevatorCal.ELEVATOR_MOTOR_ENCODER_IPS_PER_RPM));

    errors +=
        SparkMaxUtils.check(
            leftMotor.setSoftLimit(
                SoftLimitDirection.kForward, ElevatorCal.ELEVATOR_POSITIVE_LIMIT_INCHES));
    errors += SparkMaxUtils.check(leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true));
    errors +=
        SparkMaxUtils.check(
            leftMotor.setSoftLimit(
                SoftLimitDirection.kReverse, ElevatorCal.ELEVATOR_NEGATIVE_LIMIT_INCHES));
    errors += SparkMaxUtils.check(leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true));

    errors += SparkMaxUtils.check(leftMotor.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(rightMotor.setIdleMode(IdleMode.kBrake));

    errors +=
        SparkMaxUtils.check(
            leftMotor.setSmartCurrentLimit(ElevatorCal.ELEVATOR_CURRENT_LIMIT_AMPS));
    errors +=
        SparkMaxUtils.check(
            rightMotor.setSmartCurrentLimit(ElevatorCal.ELEVATOR_CURRENT_LIMIT_AMPS));

    return errors == 0;
  }

  public void burnFlashSparks() {
    Timer.delay(0.005);
    leftMotor.burnFlash();
    Timer.delay(0.005);
    rightMotor.burnFlash();
  }
  
}

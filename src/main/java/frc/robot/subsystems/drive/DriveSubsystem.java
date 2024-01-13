package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;

  // Create SwerveModules
  public final SwerveModule frontLeft = new SwerveModule(
      RobotMap.FRONT_LEFT_DRIVING_CAN_ID,
      RobotMap.FRONT_LEFT_TURNING_CAN_ID,
      DriveCal.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule frontRight = new SwerveModule(
      RobotMap.FRONT_RIGHT_DRIVING_CAN_ID,
      RobotMap.FRONT_RIGHT_TURNING_CAN_ID,
      DriveCal.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearLeft = new SwerveModule(
      RobotMap.REAR_LEFT_DRIVING_CAN_ID,
      RobotMap.REAR_LEFT_TURNING_CAN_ID,
      DriveCal.BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearRight = new SwerveModule(
      RobotMap.REAR_RIGHT_DRIVING_CAN_ID,
      RobotMap.REAR_RIGHT_TURNING_CAN_ID,
      DriveCal.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);
  
  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.PIGEON_CAN_ID);
}
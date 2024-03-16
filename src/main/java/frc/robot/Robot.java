// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intakeLimelight.IntakeLimelightConstants;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooterLimelight.ShooterLimelightConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.MatchStateUtil;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private SwerveDrivePoseEstimator m_PoseEstimator;

  private MatchStateUtil matchState = new MatchStateUtil(false, true);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /**
     * Instantiate our RobotContainer. This will perform all our button bindings, and put our
     * autonomous chooser on the dashboard.
     */
    DataLogManager.start();
    URCL.start();

    m_robotContainer = new RobotContainer(matchState);
    RobotController.setBrownoutVoltage(Constants.BROWNOUT_VOLTAGE);
    m_PoseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            m_robotContainer.drive.getGyro().getRotation2d(),
            m_robotContainer.drive.getModulePositions(),
            new Pose2d());

    m_robotContainer.isTeleop = false;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.isTeleop = false;
    LimelightHelpers.getLatestResults(
        IntakeLimelightConstants.INTAKE_LIMELIGHT_NAME); // It takes 2.5-3s on first run
    LimelightHelpers.getLatestResults(ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME);

    if (!matchState.isRealMatch()) {
      m_robotContainer.intake.pivotMotor.setIdleMode(IdleMode.kCoast);
      m_robotContainer.elevator.leftMotor.setIdleMode(IdleMode.kCoast);
      m_robotContainer.elevator.rightMotor.setIdleMode(IdleMode.kCoast);
      m_robotContainer.shooter.pivotMotor.setIdleMode(IdleMode.kCoast);

      m_robotContainer.drive.frontRight.drivingTalon.setNeutralMode(NeutralModeValue.Coast);
      m_robotContainer.drive.frontLeft.drivingTalon.setNeutralMode(NeutralModeValue.Coast);
      m_robotContainer.drive.rearRight.drivingTalon.setNeutralMode(NeutralModeValue.Coast);
      m_robotContainer.drive.rearLeft.drivingTalon.setNeutralMode(NeutralModeValue.Coast);

      m_robotContainer.drive.frontRight.turningSparkMax.setIdleMode(IdleMode.kCoast);
      m_robotContainer.drive.frontLeft.turningSparkMax.setIdleMode(IdleMode.kCoast);
      m_robotContainer.drive.rearRight.turningSparkMax.setIdleMode(IdleMode.kCoast);
      m_robotContainer.drive.rearLeft.turningSparkMax.setIdleMode(IdleMode.kCoast);
    }

    m_robotContainer.shooter.setShooterMode(ShooterMode.IDLE);
    Conveyor.stop(m_robotContainer.conveyor);
    m_robotContainer.intake.stopRollers();

    m_robotContainer.intake.dontAllowIntakeMovement();
    m_robotContainer.elevator.dontAllowElevatorMovement();
    m_robotContainer.shooter.dontAllowShooterMovement();
  }

  @Override
  public void disabledPeriodic() {
    if (!matchState.isRealMatch()) {
      m_robotContainer.shooter.considerZeroingEncoder();
      m_robotContainer.intake.considerZeroingEncoder();
    }
    m_robotContainer.shooterLimelight.resetOdometryWithTags(
        m_PoseEstimator, m_robotContainer.drive);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.isTeleop = false;

    setMatchState();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null && m_robotContainer.getAutonomousName() != null) {
      if (m_robotContainer.shooterLimelight.checkForTag().isEmpty()) {
        Pose2d pathStartingPose =
            PathPlannerAuto.getStaringPoseFromAutoFile(m_robotContainer.getAutonomousName());
        if (Math.abs(m_robotContainer.drive.getPose().getX()) < Constants.ODOMETRY_ERROR
            && Math.abs(m_robotContainer.drive.getPose().getY()) < Constants.ODOMETRY_ERROR) {
          if (matchState.isRed()) {
            Pose2d flippedPose = GeometryUtil.flipFieldPose(pathStartingPose);
            m_robotContainer.drive.resetOdometry(flippedPose);
            m_robotContainer.drive.resetYawToAngle(flippedPose.getRotation().getDegrees());
          } else {
            m_robotContainer.drive.resetOdometry(pathStartingPose);
            m_robotContainer.drive.resetYawToAngle(pathStartingPose.getRotation().getDegrees());
          }
        }
      }
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    if (matchState.isRealMatch()) {
      m_robotContainer.intake.pivotMotor.setIdleMode(IdleMode.kBrake);
      m_robotContainer.elevator.leftMotor.setIdleMode(IdleMode.kBrake);
      m_robotContainer.elevator.rightMotor.setIdleMode(IdleMode.kBrake);
      m_robotContainer.shooter.pivotMotor.setIdleMode(IdleMode.kBrake);

      m_robotContainer.drive.frontRight.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
      m_robotContainer.drive.frontLeft.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
      m_robotContainer.drive.rearRight.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
      m_robotContainer.drive.rearLeft.drivingTalon.setNeutralMode(NeutralModeValue.Brake);

      m_robotContainer.drive.frontRight.turningSparkMax.setIdleMode(IdleMode.kBrake);
      m_robotContainer.drive.frontLeft.turningSparkMax.setIdleMode(IdleMode.kBrake);
      m_robotContainer.drive.rearRight.turningSparkMax.setIdleMode(IdleMode.kBrake);
      m_robotContainer.drive.rearLeft.turningSparkMax.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.intake.stopRollers();
    Conveyor.stop(m_robotContainer.conveyor);
    m_robotContainer.shooter.setShooterMode(ShooterMode.IDLE);
    m_robotContainer.drive.frontRight.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.drive.frontLeft.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.drive.rearRight.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.drive.rearLeft.drivingTalon.setNeutralMode(NeutralModeValue.Brake);

    m_robotContainer.drive.frontRight.turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_robotContainer.drive.frontLeft.turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_robotContainer.drive.rearRight.turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_robotContainer.drive.rearLeft.turningSparkMax.setIdleMode(IdleMode.kBrake);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.lights.toggleCode(LightCode.NOTELESS);
    m_robotContainer.intake.stopRollers();
    m_robotContainer.conveyor.stopRollers();
    m_robotContainer.shooter.setShooterMode(ShooterMode.IDLE);

    setMatchState();

    m_robotContainer.isTeleop = true;

    m_robotContainer.intake.pivotMotor.setIdleMode(IdleMode.kBrake);
    m_robotContainer.elevator.leftMotor.setIdleMode(IdleMode.kBrake);
    m_robotContainer.elevator.rightMotor.setIdleMode(IdleMode.kBrake);
    m_robotContainer.shooter.pivotMotor.setIdleMode(IdleMode.kBrake);

    m_robotContainer.drive.frontRight.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.drive.frontLeft.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.drive.rearRight.drivingTalon.setNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.drive.rearLeft.drivingTalon.setNeutralMode(NeutralModeValue.Brake);

    m_robotContainer.drive.frontRight.turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_robotContainer.drive.frontLeft.turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_robotContainer.drive.rearRight.turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_robotContainer.drive.rearLeft.turningSparkMax.setIdleMode(IdleMode.kBrake);

    // if (!matchState.isRealMatch()) {
    //   if (m_robotContainer.shooterLimelight.checkForTag().isEmpty()) {
    //     Pose2d pathStartingPose =
    //         PathPlannerAuto.getStaringPoseFromAutoFile(m_robotContainer.getAutonomousName());
    //     if (Math.abs(m_robotContainer.drive.getPose().getX()) < Constants.ODOMETRY_ERROR
    //         && Math.abs(m_robotContainer.drive.getPose().getY()) < Constants.ODOMETRY_ERROR) {
    //       if (matchState.isRed()) {
    //         Pose2d flippedPose = GeometryUtil.flipFieldPose(pathStartingPose);
    //         m_robotContainer.drive.resetOdometry(flippedPose);
    //         m_robotContainer.drive.resetYawToAngle(flippedPose.getRotation().getDegrees());
    //       } else {
    //         m_robotContainer.drive.resetOdometry(pathStartingPose);
    //         m_robotContainer.drive.resetYawToAngle(pathStartingPose.getRotation().getDegrees());
    //       }
    //     }
    //   }
    // }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
   * Sets whether it is a real match by checking if the match time is more than 1s, and whether we
   * are blue from the driver station data (defaults to true).
   */
  private void setMatchState() {
    boolean realMatch = DriverStation.getMatchTime() > 1.0;
    matchState.setRealMatch(realMatch);
    if (DriverStation.getAlliance().isPresent()) {
      boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
      matchState.setBlue(isBlue);
    } else {
      matchState.setBlue(true);
    }
  }
}

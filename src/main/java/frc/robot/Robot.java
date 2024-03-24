// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.RobotContainer.PrepState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.SwerveModule;
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

  private MatchStateUtil matchState = new MatchStateUtil(false, true, false);

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
    DriverStation.startDataLog(DataLogManager.getLog()); // log joystick data
    URCL.start();

    m_robotContainer = new RobotContainer(matchState);
    RobotController.setBrownoutVoltage(Constants.BROWNOUT_VOLTAGE);
    m_PoseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            m_robotContainer.drive.getGyro().getRotation2d(),
            m_robotContainer.drive.getModulePositions(),
            new Pose2d());
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
    matchState.setTeleop(false);

    m_robotContainer.lights.setLEDColor(LightCode.DISABLED);
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
    m_robotContainer.conveyor.stopRollers();
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
      m_robotContainer.drive.considerZeroingSwerveEncoders();
    }
    m_robotContainer.shooterLimelight.resetOdometryWithTags(
        m_PoseEstimator, m_robotContainer.drive);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    matchState.updateMatchState(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    SwerveModule[] moduleArray = {
      m_robotContainer.drive.frontLeft,
      m_robotContainer.drive.frontRight,
      m_robotContainer.drive.rearLeft,
      m_robotContainer.drive.rearRight
    };
    for (var module : moduleArray) {
      var talon = module.drivingTalon;
      var currentConfig = module.appliedConfiguration;
      var conigurator = talon.getConfigurator();
      currentConfig.CurrentLimits.SupplyCurrentLimit = 50;
      conigurator.apply(currentConfig);
    }

    // If there's a path planner auto and the robot didn't initialize its pose from tags, then
    // initialize from the path's starting pose
    if (m_autonomousCommand != null && m_robotContainer.getAutonomousName() != null) {
      if (m_robotContainer.shooterLimelight.checkForTag().isEmpty()) {
        Pose2d pathStartingPose =
            PathPlannerAuto.getStaringPoseFromAutoFile(m_robotContainer.getAutonomousName());
        if (Math.abs(m_robotContainer.drive.getPose().getX())
                < Constants.ODOMETRY_MARGIN_FOR_ZEROING_M
            && Math.abs(m_robotContainer.drive.getPose().getY())
                < Constants.ODOMETRY_MARGIN_FOR_ZEROING_M) {
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

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.intake.stopRollers();
    m_robotContainer.conveyor.stopRollers();
    m_robotContainer.shooter.setShooterMode(ShooterMode.IDLE);
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

    SwerveModule[] moduleArray = {
      m_robotContainer.drive.frontLeft,
      m_robotContainer.drive.frontRight,
      m_robotContainer.drive.rearLeft,
      m_robotContainer.drive.rearRight
    };
    for (var module : moduleArray) {
      var talon = module.drivingTalon;
      var currentConfig = module.appliedConfiguration;
      var conigurator = talon.getConfigurator();
      currentConfig.CurrentLimits.SupplyCurrentLimit = 40;
      conigurator.apply(currentConfig);
    }

    m_robotContainer.lights.setLEDColor(
                        !m_robotContainer.conveyor.backConveyorBeamBreakSensor.get() ?
                        LightCode.HAS_NOTE :
                        LightCode.OFF);

    m_robotContainer.intake.stopRollers();
    m_robotContainer.conveyor.stopRollers();
    m_robotContainer.shooter.setShooterMode(ShooterMode.IDLE);

    matchState.updateMatchState(true);

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

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_robotContainer.readyToScoreCheck()) {
      m_robotContainer.lights.setLEDColor(LightCode.READY_TO_SCORE);
    }
  }

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
}

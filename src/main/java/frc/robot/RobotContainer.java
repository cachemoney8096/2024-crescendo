// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.JoystickUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * This class stores the state of the match (the alliance color, and whether it's a real match)
   *
   * @param realMatch a boolean variable defining whether we are in a real match; this can be
   *     determined by checking if the remaining match time in the init function for the match face
   *     is greater than one second
   * @param blue a boolean variable defining whether we are on the blue alliance (true) or the red
   *     alliance (false). When in a fake match, this variable should be true.
   */
  public class MatchState {
    public boolean realMatch;
    public boolean blue;

    public MatchState(boolean realMatch, boolean blue) {
      this.realMatch = realMatch;
      this.blue = blue;
    }
  }

  public MatchState matchState = new MatchState(false, true);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);

  public DriveSubsystem drive;
  public Intake intake;
  public Elevator elevator;
  public Shooter shooter;
  public Conveyor conveyor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add subsystems
    drive = new DriveSubsystem(matchState);
    intake = new Intake();
    elevator = new Elevator();
    shooter = new Shooter();
    conveyor = new Conveyor();

    // Configure the trigger bindings
    configureBindings();

    Shuffleboard.getTab("Subsystems").add(drive.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(intake.getName(), intake);
    Shuffleboard.getTab("Subsystems").add(conveyor.getName(), conveyor);
    Shuffleboard.getTab("Subsystems").add(shooter.getName(), shooter);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);

    burnFlashAllSparks();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  private void burnFlashAllSparks() {
    Timer.delay(0.25);
    drive.burnFlashSparks();
    intake.burnFlashSpark();
    conveyor.burnFlashSparks();
    elevator.burnFlashSparks();
    shooter.burnFlashSparks();
    Timer.delay(0.25);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

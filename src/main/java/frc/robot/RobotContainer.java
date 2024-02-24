// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpPrepScore;
import frc.robot.commands.AmpScore;
import frc.robot.commands.ClimbPrepSequence;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.RotateToSpeaker;
import frc.robot.commands.SpeakerPrepScoreSequence;
import frc.robot.commands.SpeakerShootSequence;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
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

  Command rumbleBriefly =
      new SequentialCommandGroup(
          new InstantCommand(
              () -> {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
              }),
          new WaitCommand(0.25),
          new InstantCommand(
              () -> {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
              }));

  public DriveSubsystem drive;
  public Intake intake;
  public Elevator elevator;
  public Shooter shooter;
  public Conveyor conveyor;
  public Lights lights;
  public ShooterLimelight shooterLimelight;

  public boolean speakerPrepped;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add subsystems
    drive = new DriveSubsystem(matchState);
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();
    conveyor = new Conveyor();
    lights = new Lights();
    shooterLimelight = new ShooterLimelight(25, Units.inchesToMeters(26), 1.45, matchState);

    // Configure the trigger bindings
    configureBindings();

    Shuffleboard.getTab("Subsystems").add(drive.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(intake.getName(), intake);
    Shuffleboard.getTab("Subsystems").add(conveyor.getName(), conveyor);
    Shuffleboard.getTab("Subsystems").add(shooter.getName(), shooter);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);
    Shuffleboard.getTab("Subsystems").add("Shooter limelight",shooterLimelight);

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
  private void configureBindings() {
    driverController
        .leftTrigger()
        .whileTrue(new IntakeSequence(intake, elevator, conveyor, shooter));
    driverController
        .leftTrigger()
        .onFalse(new GoHomeSequence(intake, elevator, shooter, conveyor, speakerPrepped));
    driverController
        .rightTrigger()
        .onTrue(
            new ConditionalCommand(
                new SpeakerShootSequence(conveyor, shooter).finallyDo(conveyor::stopRollers),
                new AmpScore(conveyor, intake, elevator),
                this::preppedSpeaker)); // score based on prep
    driverController
        .leftBumper()
        .onTrue(
            new SequentialCommandGroup(
                new SpeakerPrepScoreSequence(intake, elevator, shooter, conveyor),
                new InstantCommand(() -> setSpeakerPrep(true))));
    driverController
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                new AmpPrepScore(elevator, conveyor, intake, shooter),
                new InstantCommand(() -> setSpeakerPrep(false))));
    driverController.back().onTrue(new GoHomeSequence(intake, elevator, shooter, conveyor, false));
    driverController.start().onTrue(new InstantCommand(drive::resetYaw));
    driverController.y().onTrue(new ClimbPrepSequence(intake, elevator, shooter));
    driverController.b().onTrue(new ClimbSequence(intake, elevator, shooter, conveyor));
    driverController.a().whileTrue(new RotateToSpeaker(drive, shooterLimelight));

    drive.setDefaultCommand(
        new RunCommand(
                () ->
                    drive.rotateOrKeepHeading(
                        MathUtil.applyDeadband(-driverController.getRightY(), 0.1),
                        MathUtil.applyDeadband(-driverController.getRightX(), 0.1),
                        JoystickUtil.squareAxis(
                            MathUtil.applyDeadband(-driverController.getLeftX(), 0.05)),
                        true, // always field relative
                        driverController.getHID().getPOV()),
                drive)
            .withName("Manual Drive"));
  }

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
   * @param true if speaker was just prepped, false if amp was just prepped
   */
  private void setSpeakerPrep(boolean prepSpeaker) {
    this.speakerPrepped = prepSpeaker;
  }

  /**
   * @return true if speaker was prepped, false if amp was prepped
   */
  private boolean preppedSpeaker() {
    return this.speakerPrepped;
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpPrepScore;
import frc.robot.commands.AmpScore;
import frc.robot.commands.ClimbPrepSequence;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.FeedPrepScore;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.PIDToPoint;
import frc.robot.commands.SetTrapLineupPosition;
import frc.robot.commands.SpeakerPrepScoreAuto;
import frc.robot.commands.SpeakerPrepScoreAutoPreload;
import frc.robot.commands.SpeakerPrepScoreSequence;
import frc.robot.commands.SpeakerShootSequence;
import frc.robot.commands.UnclimbSequence;
import frc.robot.commands.autos.ScoreFourFromCenterLine;
import frc.robot.commands.autos.ScoreTwoNotes;
import frc.robot.commands.autos.TwoWithCenterNote;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeCal;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.intakeLimelight.IntakeLimelightConstants;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import frc.robot.subsystems.shooterLimelight.ShooterLimelightConstants;
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

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public DriveSubsystem drive;
  public Intake intake;
  public Elevator elevator;
  public Shooter shooter;
  public Conveyor conveyor;
  public Lights lights;
  public ShooterLimelight shooterLimelight;
  public IntakeLimelight intakeLimelight;

  public boolean speakerPrepped;
  public boolean driveFieldRelative = true;
  public boolean feedPrepped = false;

  // A chooser for autonomous commands
  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add subsystems
    drive = new DriveSubsystem(matchState);
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();
    conveyor =
        new Conveyor(
            (double val) -> {
              driverController.getHID().setRumble(RumbleType.kBothRumble, val);
            });
    lights = new Lights();
    shooterLimelight =
        new ShooterLimelight(
            ShooterLimelightConstants.SHOOTER_LIMELIGHT_PITCH_DEGREES,
            ShooterLimelightConstants.SHOOTER_LIMELIGHT_HEIGHT_METERS,
            ShooterLimelightConstants.SHOOTER_LIMELIGHT_TARGET_HEIGHT_METERS,
            matchState);
    intakeLimelight =
        new IntakeLimelight(
            IntakeLimelightConstants.INTAKE_LIMELIGHT_PITCH_DEGREES,
            IntakeLimelightConstants.INTAKE_LIMELIGHT_HEIGHT_METERS,
            0); // we aren't using the target height so 0 is fine

    NamedCommands.registerCommand(
        "SPEAKER PREP PRELOAD",
        new SpeakerPrepScoreAutoPreload(intake, elevator, shooter, conveyor));
    NamedCommands.registerCommand(
        "SPEAKER SCORE",
        new InstantCommand(() -> System.out.println("Speaker score starting"))
            .andThen(Conveyor.shoot(conveyor))
            .andThen(new InstantCommand(() -> System.out.println("Speaker Score Done"))));
    NamedCommands.registerCommand(
        "INTAKE DEPLOY",
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)));
    NamedCommands.registerCommand(
        "INTAKE", new IntakeSequence(intake, elevator, conveyor, shooter));
    NamedCommands.registerCommand(
        "SPEAKER PREP", new SpeakerPrepScoreAuto(intake, elevator, shooter, conveyor));
    // Configure the controller bindings
    configureDriver();
    configureOperator();

    Shuffleboard.getTab("Subsystems").add(drive.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(intake.getName(), intake);
    Shuffleboard.getTab("Subsystems").add(conveyor.getName(), conveyor);
    Shuffleboard.getTab("Subsystems").add(shooter.getName(), shooter);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);
    Shuffleboard.getTab("Subsystems").add("Shooter limelight", shooterLimelight);
    Shuffleboard.getTab("Subsystems").add("Intake limelight", intakeLimelight);

    SmartDashboard.putBoolean("Have Note", false);

    burnFlashAllSparks();

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    autonChooser.setDefaultOption(
        "Score two",
        new ScoreTwoNotes(
            intake, elevator, shooter, conveyor, drive, matchState, shooterLimelight));
    autonChooser.addOption(
        "Score four from center",
        new ScoreFourFromCenterLine(
            matchState.blue, drive, intake, elevator, shooter, conveyor, shooterLimelight));
    autonChooser.addOption(
        "Score two with center note",
        new TwoWithCenterNote(drive, intake, elevator, shooter, conveyor, shooterLimelight));
  }

  private int getCardinalDirectionDegrees() {
    if (driverController.getHID().getAButton()) {
      return matchState.blue ? 180 : 0;
    } else if (driverController.getHID().getBButton()) {
      return matchState.blue ? 90 : 270;
    } else if (driverController.getHID().getXButton()) {
      return matchState.blue ? 270 : 90;
    } else if (driverController.getHID().getYButton()) {
      return matchState.blue ? 0 : 180;
    } else {
      return -1;
    }
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
  private void configureDriver() {
    driverController
        .rightTrigger()
        .whileTrue(new IntakeSequence(intake, elevator, conveyor, shooter));
    driverController
        .rightTrigger()
        .onFalse(new GoHomeSequence(intake, elevator, shooter, conveyor, false, false));
    driverController
        .leftTrigger()
        .onTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    new SpeakerShootSequence(conveyor, shooter, elevator, drive, !feedPrepped)
                        .beforeStarting(
                            () -> {
                              System.out.println(feedPrepped);
                            })
                        .andThen(new InstantCommand(() -> shooter.readyToShoot = false))
                        .andThen(
                            new InstantCommand(
                                () -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)))
                        .finallyDo(conveyor::stopRollers),
                    new InstantCommand(),
                    () -> shooter.readyToShoot),
                new AmpScore(drive, conveyor, intake, shooter, elevator),
                this::preppedSpeaker)); // score based on prep
    driverController
        .leftBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setSpeakerPrep(true)),
                new InstantCommand(() -> feedPrepped = false),
                new SpeakerPrepScoreSequence(
                    intake, elevator, shooter, conveyor, shooterLimelight, drive)));
    driverController
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                new AmpPrepScore(elevator, conveyor, intake, shooter),
                new InstantCommand(() -> setSpeakerPrep(false))));
    // bottom right back button
    driverController
        .povLeft()
        .onTrue(
            new GoHomeSequence(intake, elevator, shooter, conveyor, false, true)
                .beforeStarting(() -> driveFieldRelative = true)
                .beforeStarting(() -> drive.throttle(1.0)));
    driverController.start().onTrue(new InstantCommand(drive::resetYaw));
    // top right button
    driverController
        .povDown()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> feedPrepped = true),
                new FeedPrepScore(elevator, conveyor, intake, shooter, drive, matchState),
                new InstantCommand(() -> setSpeakerPrep(true))));
    // bottom left back button
    driverController.povRight().onTrue(new ClimbSequence(intake, elevator, shooter, conveyor));

    // top left back button
    driverController
        .povUp()
        .onTrue(
            new ClimbPrepSequence(intake, elevator, shooter, conveyor, intakeLimelight)
                .andThen(new WaitUntilCommand(() -> elevator.atDesiredPosition()))
                // .andThen(new SetTrapLineupPosition(intakeLimelight, drive).withTimeout(4.0)));
                .andThen(
                    new SequentialCommandGroup(
                            new SetTrapLineupPosition(intakeLimelight, drive),
                            new PIDToPoint(drive))
                        .withTimeout(4.0))
                .andThen(new InstantCommand(() -> driveFieldRelative = false))
                .andThen(new InstantCommand(() -> drive.throttle(0.3))));

    driverController.back().onTrue(new UnclimbSequence(elevator, shooter));

    drive.setDefaultCommand(
        new RunCommand(
                () -> {
                  if (matchState.blue) {
                    drive.rotateOrKeepHeading(
                        MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                        MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                        JoystickUtil.squareAxis(
                            MathUtil.applyDeadband(-driverController.getRightX(), 0.05)),
                        driveFieldRelative, // always field relative
                        getCardinalDirectionDegrees());
                  } else {
                    if (driveFieldRelative) {
                      drive.rotateOrKeepHeading(
                          MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                          MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                          JoystickUtil.squareAxis(
                              MathUtil.applyDeadband(-driverController.getRightX(), 0.05)),
                          driveFieldRelative, // always field relative
                          getCardinalDirectionDegrees());
                    } else {
                      drive.rotateOrKeepHeading(
                          MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                          MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                          JoystickUtil.squareAxis(
                              MathUtil.applyDeadband(-driverController.getRightX(), 0.05)),
                          driveFieldRelative, // always field relative
                          getCardinalDirectionDegrees());
                    }
                  }
                },
                drive)
            .withName("Manual Drive"));
  }

  private void configureOperator() {
    operatorController.x().onTrue(new InstantCommand(() -> conveyor.startRollers(1.0)));
    operatorController.x().onFalse(new InstantCommand(() -> conveyor.stopRollers()));
    operatorController
        .b()
        .onTrue(
            new InstantCommand(() -> conveyor.startRollers(-1.0))
                .andThen(() -> intake.reverseRollers()));
    operatorController
        .b()
        .onFalse(
            new InstantCommand(() -> conveyor.stopRollers()).andThen(() -> intake.stopRollers()));
    operatorController.a().onTrue(new InstantCommand(() -> shooter.setShooterDistance(1.16)));
    operatorController.y().onTrue(new InstantCommand(() -> shooter.setShooterDistance(2.77)));

    operatorController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                    () -> intake.rezeroIntakeToPosition(IntakeCal.INTAKE_STOWED_POSITION_DEGREES))
                .ignoringDisable(true));
    operatorController
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                    () -> intake.rezeroIntakeToPosition(IntakeCal.INTAKE_DEPLOYED_POSITION_DEGREES))
                .ignoringDisable(true));
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
    // elevator currently goes UP in auto (score two notes)!!

    return autonChooser.getSelected();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpPrepScore;
import frc.robot.commands.AmpScore;
import frc.robot.commands.AutoIntakeSequence;
import frc.robot.commands.ClimbPrepSequence;
import frc.robot.commands.ClimbSequence;
import frc.robot.commands.FeedPrepScore;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.PIDToPoint;
import frc.robot.commands.PartialClimbSequence;
import frc.robot.commands.SetTrapLineupPosition;
import frc.robot.commands.SpeakerPrepScoreAuto;
import frc.robot.commands.SpeakerPrepScoreAutoPreload;
import frc.robot.commands.SpeakerPrepScoreSequence;
import frc.robot.commands.SpeakerShootSequence;
import frc.robot.commands.UnclimbSequence;
import frc.robot.commands.autos.CenterOneToFive;
import frc.robot.commands.autos.ScoreFourFromCenterLine;
import frc.robot.commands.autos.ScoreTwoNotes;
import frc.robot.commands.autos.SideSixToEight;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorCal;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeCal;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.intakeLimelight.IntakeLimelightConstants;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterCal;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import frc.robot.subsystems.shooterLimelight.ShooterLimelightConstants;
import frc.robot.utils.JoystickUtil;
import frc.robot.utils.MatchStateUtil;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable {
  private MatchStateUtil matchState;

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

  public enum PrepState {
    OFF,
    CLIMB,
    SPEAKER,
    FEED,
    AMP,
    OPERATOR_PREPPED
  }

  public boolean isTeleop = false;

  PrepState prepState = PrepState.OFF;

  public boolean driveFieldRelative = true;

  public String pathCmd = "";

  // A chooser for autonomous commands
  private SendableChooser<Pair<Command, String>> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(MatchStateUtil matchState) {
    this.matchState = matchState;

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
        new InstantCommand(() -> pathCmd = "SPEAKER PREP PRELOAD")
            .andThen(new SpeakerPrepScoreAutoPreload(intake, elevator, shooter, conveyor, ShooterCal.AUTO_PRELOAD_DISTANCE_M))
            .andThen(new InstantCommand(() -> System.out.println("speaker prep preload - done"))));
    // for when we start a little bit off of the subwoofer but actually we aren't doing that anymore but im not gonna remove the code rn, cope 🤷
    NamedCommands.registerCommand(
        "SPEAKER PREP PRELOAD 2",
        new InstantCommand(() -> pathCmd = "SPEAKER PREP PRELOAD 2")
            .andThen(new SpeakerPrepScoreAutoPreload(intake, elevator, shooter, conveyor, ShooterCal.AUTO_PRELOAD_DISTANCE_2_M))
            .andThen(new InstantCommand(() -> System.out.println("speaker prep preload - done"))));
    NamedCommands.registerCommand(
        "SPEAKER SCORE",
        new SequentialCommandGroup(
            new InstantCommand(() -> pathCmd = "SPEAKER SCORE"),
            new InstantCommand(() -> System.out.println("Speaker score starting")),
            new WaitCommand(0.4),
            Conveyor.shoot(conveyor),
            new InstantCommand(() -> System.out.println("Speaker Score Done"))));
    NamedCommands.registerCommand(
        "INTAKE DEPLOY",
        new InstantCommand(() -> pathCmd = "INTAKE DEPLOY")
            .andThen(
                new InstantCommand(
                    () -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED))));
    NamedCommands.registerCommand(
        "INTAKE",
        new InstantCommand(() -> pathCmd = "INTAKE")
            .andThen(new AutoIntakeSequence(intake, elevator, conveyor, lights)).finallyDo(() -> new InstantCommand(() -> conveyor.stopRollers())));
    NamedCommands.registerCommand(
        "SPEAKER PREP",
        new InstantCommand(() -> pathCmd = "SPEAKER PREP")
            .andThen(
                new SpeakerPrepScoreAuto(
                    intake, elevator, shooter, conveyor, ShooterCal.AUTO_SHOOTING_DISTANCE_M))
            .andThen(
                new InstantCommand(
                    () -> System.out.println("LSDGJDLFKGJDLFKGJLDLKGJDLFGJDLTGKJDLKGJDFLKGJ"))));
    NamedCommands.registerCommand(
        "SPEAKER PREP STAGE",
        new InstantCommand(() -> pathCmd = "SPEAKER PREP STAGE")
            .andThen(new InstantCommand(() -> conveyor.stopRollers()))
            .andThen(
                new SpeakerPrepScoreAuto(
                    intake, elevator, shooter, conveyor, ShooterCal.AUTO_STAGE_SHOOTING_DISTANCE_M))
            .andThen(
                new InstantCommand(
                    () -> System.out.println("LSDGJDLFKGJDLFKGJLDLKGJDLFGJDLTGKJDLKGJDFLKGJ"))));
    NamedCommands.registerCommand(
        "CONVEYOR OVERRIDE",
        new InstantCommand(() -> pathCmd = "CONVEYOR OVERRIDE")
            .andThen(Conveyor.stop(conveyor))
            .andThen(new InstantCommand(() -> shooter.setShooterMode(ShooterMode.IDLE)))
            .andThen(new InstantCommand(() -> intake.stopRollers())));

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
    Shuffleboard.getTab("Subsystems").add("Container", this);
    Shuffleboard.getTab("Subsystems").add("This is a duplicate elevator subsystem", elevator);

    SmartDashboard.putBoolean("Have Note", false);

    burnFlashAllSparks();

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    autonChooser.setDefaultOption(
        "Score two",
        new Pair<Command, String>(
            new ScoreTwoNotes(
                intake, elevator, shooter, conveyor, drive, matchState, shooterLimelight, lights),
            null));
    autonChooser.addOption(
        "Score four from center",
        new Pair<Command, String>(
            new ScoreFourFromCenterLine(
                drive, intake, elevator, shooter, conveyor, shooterLimelight),
            "4 NOTE - CENTER LINE - SOURCE - AUTO"));
    autonChooser.addOption(
        "BLUE score notes 1-2-3-4-5",
        new Pair<Command, String>(
            new CenterOneToFive(
                drive, intake, elevator, shooter, conveyor, shooterLimelight, false),
            "CENTER 1-2-3-4-5"));
    autonChooser.addOption(
        "RED score notes 1-2-3-4-5",
        new Pair<Command, String>(
            new CenterOneToFive(drive, intake, elevator, shooter, conveyor, shooterLimelight, true),
            "CENTER 1-2-3-4-5 RED"));
    autonChooser.addOption(
        "BLUE score notes 8-6-7 right",
        new Pair<Command, String>(
            new SideSixToEight(drive, intake, elevator, shooter, conveyor, shooterLimelight, false),
            "RIGHT 8-6-7"));
    autonChooser.addOption(
        "RED score notes 8-6-7 left",
        new Pair<Command, String>(
            new SideSixToEight(drive, intake, elevator, shooter, conveyor, shooterLimelight, true),
            "LEFT 8-6-7 RED"));
    SmartDashboard.putData(autonChooser);
  }

  private int getCardinalDirectionDegrees() {
    if (driverController.getHID().getAButton()) {
      return matchState.isBlue() ? 180 : 0;
    } else if (driverController.getHID().getBButton()) {
      return matchState.isBlue() ? 90 : 270;
    } else if (driverController.getHID().getXButton()) {
      return matchState.isBlue() ? 270 : 90;
    } else if (driverController.getHID().getYButton()) {
      return matchState.isBlue() ? 0 : 180;
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
        .whileTrue(new IntakeSequence(intake, elevator, conveyor, shooter, lights));
    driverController
        .rightTrigger()
        .onFalse(
            Conveyor.finishReceive(conveyor)
                .andThen(
                    new GoHomeSequence(intake, elevator, shooter, conveyor, false, false, true))
                .beforeStarting(() -> prepState = PrepState.OFF));
    driverController
        .leftTrigger()
        .onTrue(
            new DeferredCommand(
                () -> {
                  PrepState input = prepState;
                  prepState = PrepState.OFF;
                  System.out.println("we are @score button");
                  switch (input) {
                    case OFF:
                      return new SequentialCommandGroup(
                          new InstantCommand(intake::reverseRollers),
                          new InstantCommand(() -> conveyor.startRollers(-1.0)),
                          new WaitCommand(ConveyorCal.NOTE_EXIT_TIME_OUTTAKE_SECONDS),
                          new InstantCommand(conveyor::stopRollers),
                          new WaitCommand(ConveyorCal.NOTE_EXIT_TIME_OUTTAKE_SECONDS),
                          new InstantCommand(intake::stopRollers));
                    case CLIMB:
                      return new ClimbSequence(intake, elevator, shooter, conveyor);
                    case FEED:
                      return new SpeakerShootSequence(conveyor, shooter, elevator, drive, false);
                    case SPEAKER:
                      return new SpeakerShootSequence(conveyor, shooter, elevator, drive, true);
                    case AMP:
                      return new AmpScore(drive, conveyor, intake, shooter, elevator);
                    case OPERATOR_PREPPED:
                      return Conveyor.shoot(conveyor);
                    default:
                      return new InstantCommand(() -> System.out.println("you suck at coding"));
                  }
                },
                new TreeSet<Subsystem>()));

    BooleanSupplier driverRotationCommanded =
        () -> {
          return Math.abs(driverController.getRightX()) > 0.05;
        };

    driverController
        .leftBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> prepState = PrepState.SPEAKER),
                new SpeakerPrepScoreSequence(
                    intake,
                    elevator,
                    shooter,
                    conveyor,
                    shooterLimelight,
                    drive,
                    driverRotationCommanded)));
    driverController
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> prepState = PrepState.AMP),
                new AmpPrepScore(elevator, conveyor, intake, shooter, drive)));
    // bottom right back button
    driverController
        .povLeft()
        .onTrue(
            new GoHomeSequence(intake, elevator, shooter, conveyor, false, true, true)
                .beforeStarting(() -> driveFieldRelative = true)
                .beforeStarting(() -> drive.throttle(1.0))
                .beforeStarting(() -> prepState = PrepState.OFF));
    driverController.start().onTrue(new InstantCommand(drive::resetYaw));
    // top right button
    driverController
        .povDown()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> prepState = PrepState.FEED),
                new FeedPrepScore(elevator, conveyor, intake, shooter, drive, matchState)));
    // bottom left back button
    driverController.povRight().onTrue(new UnclimbSequence(elevator, shooter, conveyor));

    BooleanSupplier driverJoysticksActive =
        () -> {
          return Math.abs(driverController.getLeftY()) > 0.2
              || Math.abs(driverController.getLeftX()) > 0.2
              || Math.abs(driverController.getRightY()) > 0.2
              || Math.abs(driverController.getRightX()) > 0.2;
        };

    // top left back button
    driverController
        .povUp()
        .onTrue(
            new InstantCommand(() -> prepState = PrepState.CLIMB)
                .andThen(
                    new ClimbPrepSequence(intake, elevator, shooter, conveyor, intakeLimelight))
                .andThen(new WaitUntilCommand(() -> elevator.atDesiredPosition()))
                // .andThen(new SetTrapLineupPosition(intakeLimelight, drive).withTimeout(4.0)));
                .andThen(
                    () ->
                        new SequentialCommandGroup(
                                new SetTrapLineupPosition(intakeLimelight, drive),
                                new PIDToPoint(drive))
                            .raceWith(new WaitUntilCommand(driverJoysticksActive))
                            .schedule())
                .andThen(new InstantCommand(() -> driveFieldRelative = false))
                .andThen(new InstantCommand(() -> drive.throttle(0.3))));

    driverController.back().onTrue(new PartialClimbSequence(intake, elevator, shooter));

    drive.setDefaultCommand(
        new ConditionalCommand(
                new RunCommand(
                    () -> {
                      if (matchState.isBlue()) {
                        drive.rotateOrKeepHeading(
                            JoystickUtil.squareAxis(
                                MathUtil.applyDeadband(-driverController.getLeftY(), 0.1)),
                            JoystickUtil.squareAxis(
                                MathUtil.applyDeadband(-driverController.getLeftX(), 0.1)),
                            JoystickUtil.squareAxis(
                                MathUtil.applyDeadband(-driverController.getRightX(), 0.05)),
                            driveFieldRelative, // always field relative
                            getCardinalDirectionDegrees());
                      } else {
                        if (driveFieldRelative) {
                          drive.rotateOrKeepHeading(
                              JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(driverController.getLeftY(), 0.1)),
                              JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(driverController.getLeftX(), 0.1)),
                              JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(-driverController.getRightX(), 0.05)),
                              driveFieldRelative, // always field relative
                              getCardinalDirectionDegrees());
                        } else {
                          drive.rotateOrKeepHeading(
                              JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(-driverController.getLeftY(), 0.1)),
                              JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(-driverController.getLeftX(), 0.1)),
                              JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(-driverController.getRightX(), 0.05)),
                              driveFieldRelative, // always field relative
                              getCardinalDirectionDegrees());
                        }
                      }
                    },
                    drive),
                new InstantCommand(),
                () -> isTeleop)
            .withName("Manual Drive"));
  }

  private Command ParallelCommandGroup(
      InstantCommand instantCommand, InstantCommand instantCommand2) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ParallelCommandGroup'");
  }

  private Command WaitUntilCommand(double noteExitTimeTrapAmpSeconds) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'WaitUntilCommand'");
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
    operatorController
        .a()
        .onTrue(
            new InstantCommand(() -> shooter.setShooterDistance(1.16))
                .andThen(new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)))
                .andThen(new InstantCommand(() -> prepState = PrepState.OPERATOR_PREPPED)));
    operatorController
        .y()
        .onTrue(
            new InstantCommand(() -> shooter.setShooterDistance(2.77))
                .andThen(new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)))
                .andThen(new InstantCommand(() -> prepState = PrepState.OPERATOR_PREPPED)));

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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // elevator currently goes UP in auto (score two notes)!!

    return autonChooser.getSelected().getFirst();
  }

  public String getAutonomusName() {
    return autonChooser.getSelected().getSecond();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "prepState",
        () -> {
          return prepState.toString();
        },
        null);
    builder.addStringProperty("pathCmd", () -> pathCmd, null);
  }
}

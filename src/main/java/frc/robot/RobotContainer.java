// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.limelightCamMode;
import frc.robot.Constants.limelightLedMode;
import frc.robot.Constants.limelightPipeline;
import frc.robot.RobotContainer.PrepState;
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
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeCal;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.intakeLimelight.IntakeLimelightConstants;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight.NoteDetection;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterCal;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import frc.robot.subsystems.shooterLimelight.ShooterLimelightConstants;
import frc.robot.utils.JoystickUtil;
import frc.robot.utils.MatchStateUtil;

import java.time.Instant;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import javax.xml.crypto.dsig.Transform;

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
    OPERATOR
  }

  PrepState prepState = PrepState.OFF;

  public boolean driveFieldRelative = true;

  public boolean usingTagHeading = false;

  /** What was the last command the auto initiated */
  public String pathCmd = "";

  /** Translation is a unit direction for robot movement
   * Rotation is desired robot heading
   */
  private Optional<Pair<Translation2d, Rotation2d>> noteDirectionOptional = Optional.empty();
  
  /**
   * A chooser for autonomous commands. String in pair should be the path's name, and null if no
   * path
   */
  private SendableChooser<Pair<Command, String>> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(MatchStateUtil matchState) {
    this.matchState = matchState;

    // Add subsystems
    drive = new DriveSubsystem(matchState);
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();
    lights = new Lights();
    conveyor =
        new Conveyor(
            (double val) -> {
              driverController.getHID().setRumble(RumbleType.kBothRumble, val);
              operatorController.getHID().setRumble(RumbleType.kBothRumble, val);
            },
            () -> {
                lights.setLEDColor(LightCode.HAS_NOTE);
            });
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
            .andThen(
                new SpeakerPrepScoreAutoPreload(
                    intake, elevator, shooter, conveyor, ShooterCal.AUTO_PRELOAD_DISTANCE_M)));
    NamedCommands.registerCommand(
        "SPEAKER SCORE",
        new SequentialCommandGroup(
            new InstantCommand(() -> pathCmd = "SPEAKER SCORE"),
            new WaitCommand(0.25),
            Conveyor.shoot(conveyor, ConveyorCal.NOTE_EXIT_TIME_SHOOTER_AUTO_SECONDS)));
    NamedCommands.registerCommand(
        "INTAKE DEPLOY",
        new InstantCommand(() -> pathCmd = "INTAKE DEPLOY")
            .andThen(
                new InstantCommand(
                    () -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED))));
    NamedCommands.registerCommand(
        "INTAKE",
        new InstantCommand(() -> pathCmd = "INTAKE")
            .andThen(new AutoIntakeSequence(intake, elevator, conveyor, lights))
            .finallyDo(conveyor::stopRollers));
    NamedCommands.registerCommand(
        "SPEAKER PREP",
        new InstantCommand(() -> pathCmd = "SPEAKER PREP")
            .andThen(
                new SpeakerPrepScoreAuto(
                    intake, elevator, shooter, conveyor, ShooterCal.AUTO_SHOOTING_DISTANCE_M)));
    NamedCommands.registerCommand(
        "SPEAKER PREP STAGE",
        new InstantCommand(() -> pathCmd = "SPEAKER PREP STAGE")
            .andThen(new InstantCommand(() -> conveyor.stopRollers()))
            .andThen(
                new SpeakerPrepScoreAuto(
                    intake,
                    elevator,
                    shooter,
                    conveyor,
                    ShooterCal.AUTO_STAGE_SHOOTING_DISTANCE_M)));
    NamedCommands.registerCommand(
        "STOP SUBSYSTEMS",
        new InstantCommand(() -> pathCmd = "STOP SUBSYSTEMS")
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

    SmartDashboard.putBoolean("Have Note", false);

    burnFlashAllSparks();

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    autonChooser.setDefaultOption(
        "Score two",
        new Pair<Command, String>(
            new ScoreTwoNotes(
                intake,
                elevator,
                shooter,
                conveyor,
                drive,
                matchState,
                shooterLimelight,
                intakeLimelight,
                lights),
            null));
    autonChooser.addOption(
        "Score four from center",
        new Pair<Command, String>(
            new ScoreFourFromCenterLine(
                drive, intake, elevator, shooter, conveyor, shooterLimelight),
            "4 NOTE - CENTER LINE - SOURCE - AUTO"));
    autonChooser.addOption(
        "BLUE score notes 2-1-3-5-4",
        new Pair<Command, String>(
            new CenterOneToFive(
                drive, intake, elevator, shooter, conveyor, shooterLimelight, false),
            "CENTER 2-1-3-5-4"));
    autonChooser.addOption(
        "RED score notes 2-1-3-5-4",
        new Pair<Command, String>(
            new CenterOneToFive(drive, intake, elevator, shooter, conveyor, shooterLimelight, true),
            "CENTER 2-1-3-5-4 RED"));
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

  private PrepState getAndClearPrepState() {
    PrepState input = this.prepState;
    this.prepState = PrepState.OFF;
    return input;
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

   public boolean intakeLockedOn = false;
   public boolean intakeNoteTooClose = false;

  private void configureDriver() {

    BooleanSupplier driverJoysticksActive =
        () -> {
          return Math.abs(driverController.getLeftY()) > 0.2
              || Math.abs(driverController.getLeftX()) > 0.2
              || Math.abs(driverController.getRightY()) > 0.2
              || Math.abs(driverController.getRightX()) > 0.2;
        };

    driverController
        .rightBumper()
        .whileTrue(
            new IntakeSequence(intake, elevator, conveyor, shooter, lights));
    driverController
        .rightBumper()
        .onFalse(
            Conveyor.finishReceive(conveyor, lights)
                .andThen(
                    new GoHomeSequence(
                        intake, elevator, shooter, conveyor, intakeLimelight, false, false, true))                .beforeStarting(() -> prepState = PrepState.OFF));


    driverController
        .rightTrigger()
        // .whileTrue(
        //     new IntakeSequence(intake, elevator, conveyor, shooter, lights));
        .whileTrue(new ParallelDeadlineGroup(
            new IntakeSequence(intake, elevator, conveyor, shooter, lights),
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeLockedOn = false;
                    intakeNoteTooClose = false;
                    noteDirectionOptional = Optional.empty();
                    intakeLimelight.setLimelightValues(
                        limelightLedMode.OFF,
                        limelightCamMode.VISION_PROCESSING,
                        limelightPipeline.NOTE_PIPELINE
                    );
                }),
            new RunCommand(()-> {
                final boolean intakeClearOfLL = intake.clearOfLimeLight();
                Optional<frc.robot.subsystems.intakeLimelight.IntakeLimelight.NoteDetection> latestNoteDetectionOptional = Optional.empty();
                if (!intakeNoteTooClose && intakeClearOfLL)
                {
                    latestNoteDetectionOptional = intakeLimelight.getNotePos();
                }

                Runnable driveNormal = () -> {
                    if (!IntakeSequence.gotNote)
                    {
                        lights.setLEDColor(LightCode.INTAKING);
                    } else {
                        lights.setLEDColor(LightCode.HAS_NOTE);
                    }
                final var translationInputs = JoystickUtil.computeDriveXY(driverController, driveFieldRelative, matchState.isBlue());
                final var rotationInput = JoystickUtil.squareAxis(
                                MathUtil.applyDeadband(-driverController.getRightX(), 0.05));
                drive.rotateOrKeepHeading(
                            translationInputs.getFirst(),
                            translationInputs.getSecond(),
                            rotationInput,
                            driveFieldRelative, // always field relative
                            getCardinalDirectionDegrees());
                };

                if (IntakeSequence.gotNote)
                {
                    driveNormal.run();
                    return;
                }

                Consumer<Pair<Translation2d, Rotation2d>> driveToPose = (Pair<Translation2d, Rotation2d> noteDirection) -> {                
                    if (!IntakeSequence.gotNote)
                    {
                        lights.setBlink(LightCode.INTAKING);
                    } else {
                        lights.setLEDColor(LightCode.HAS_NOTE);
                    }
                    drive.setTargetHeadingDegrees(noteDirection.getSecond().getDegrees());
                    
                    var driverTranslationInput = new Translation2d(
                        JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(-driverController.getLeftX(), 0.1)),
                        JoystickUtil.squareAxis(
                                  MathUtil.applyDeadband(-driverController.getLeftY(), 0.1)));
                    var driveThrottle = driverTranslationInput.getNorm();
                    var driveTranslation = noteDirection.getFirst().times(driveThrottle);
                    drive.rotateOrKeepHeading(
                        driveTranslation.getX(),
                        driveTranslation.getY(),
                        0.0,
                        true,
                        -1);
                };

                if (intakeNoteTooClose)
                {
                    // Affect throttle with past detection
                    assert noteDirectionOptional.isPresent();
                    driveToPose.accept(noteDirectionOptional.get());
                    return;
                }

                if (latestNoteDetectionOptional.isPresent())
                {                    
                    // Set goal
                    var latestNoteDetection = latestNoteDetectionOptional.get();
                    final double adjustmentMeters = Units.inchesToMeters(6.0);
                    Translation2d poseAtDetectionToNote = new Translation2d(latestNoteDetection.distanceMeters - adjustmentMeters, 0.0).rotateBy(Rotation2d.fromDegrees(latestNoteDetection.yawAngleDeg));
                    Pose2d robotPoseAtDetection = drive.getPastBufferedPose(latestNoteDetection.latencySec);
                    Pose2d curPose = drive.getPose();
                    Pose2d goalPose = robotPoseAtDetection.plus(
                        new Transform2d(poseAtDetectionToNote, Rotation2d.fromDegrees(latestNoteDetection.yawAngleDeg)));
                    Translation2d toNoteTranslation = goalPose.getTranslation().minus(curPose.getTranslation());

                    // Keep in mind: these are robot relative
                    noteDirectionOptional = Optional.of(Pair.of(toNoteTranslation.div(toNoteTranslation.getNorm()), goalPose.getRotation()));

                    // Check distance
                    if (latestNoteDetection.distanceMeters < 1.7) {
                        intakeNoteTooClose = true;
                    }

                    // Affect throttle
                    driveToPose.accept(noteDirectionOptional.get());
                    return;
                }
                else
                {
                    if (noteDirectionOptional.isEmpty())
                    {
                        driveNormal.run();
                        return;
                    }
    
                    if (noteDirectionOptional.isPresent())
                    {
                        driveToPose.accept(noteDirectionOptional.get());
                        return;
                    }
                }
            }, drive)).finallyDo(() -> {
                
        intakeLimelight.setLimelightValues(
                Constants.limelightLedMode.OFF,
                Constants.limelightCamMode.DRIVER_CAMERA,
                Constants.limelightPipeline.NOTE_PIPELINE);
            })
            ));
    driverController
        .rightTrigger()
        .onFalse(
            Conveyor.finishReceive(conveyor, lights)
            .andThen(new InstantCommand(() -> lights.setLEDColor(
                        !conveyor.backConveyorBeamBreakSensor.get() ?
                        LightCode.HAS_NOTE :
                        LightCode.OFF)))
                .andThen(
                    new GoHomeSequence(
                        intake, elevator, shooter, conveyor, intakeLimelight, false, false, true))                .beforeStarting(() -> prepState = PrepState.OFF));


    TreeMap<PrepState, Command> selectCommandMap = new TreeMap<PrepState, Command>();
    selectCommandMap.put(
        PrepState.OFF,
        new SequentialCommandGroup(
            new InstantCommand(intake::reverseRollers),
            new InstantCommand(() -> conveyor.startRollers(-1.0)),
            new WaitCommand(ConveyorCal.NOTE_EXIT_TIME_OUTTAKE_SECONDS),
            new InstantCommand(conveyor::stopRollers),
            new WaitCommand(ConveyorCal.NOTE_EXIT_TIME_OUTTAKE_SECONDS),
            new InstantCommand(intake::stopRollers)));
    selectCommandMap.put(
        PrepState.CLIMB,
        new SequentialCommandGroup(new ClimbSequence(intake, elevator, shooter, conveyor)));
    selectCommandMap.put(
        PrepState.FEED,
        new SequentialCommandGroup(
            // new InstantCommand(() -> shooterLimelight.resetOdometryDuringPrep(drive)),
            new InstantCommand(() -> System.out.println("feeding - speaker shoot sequence is next")),
            new SpeakerShootSequence(conveyor, shooter, elevator, drive, false)));
    selectCommandMap.put(
        PrepState.SPEAKER,
        new SequentialCommandGroup(
            new SpeakerShootSequence(conveyor, shooter, elevator, drive, true)));
    selectCommandMap.put(
        PrepState.AMP,
        new SequentialCommandGroup(
            new AmpScore(drive, conveyor, intake, shooter, elevator, intakeLimelight),
            new InstantCommand(() -> drive.throttle(1.0))));
    selectCommandMap.put(PrepState.OPERATOR, new SequentialCommandGroup(Conveyor.shoot(conveyor),
    new InstantCommand(()->shooterLimelight.resetOdometryDuringPrep(drive)),
    new InstantCommand(()->System.out.println("rezeroed odemetry in speakerprep"))));

    SelectCommand<PrepState> driverLeftTriggerCommand =
        new SelectCommand<PrepState>(selectCommandMap, this::getAndClearPrepState);

    driverController
        .leftTrigger()
        .onTrue(
            driverLeftTriggerCommand.andThen(
                new InstantCommand(() -> lights.setLEDColor(LightCode.OFF))));

    BooleanSupplier driverRotationCommanded =
        () -> {
          return Math.abs(driverController.getRightX()) > 0.05;
        };

    driverController
        .leftBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> prepState = PrepState.SPEAKER),
                new InstantCommand(() -> usingTagHeading = false),
                new SpeakerPrepScoreSequence(
                    intake,
                    elevator,
                    shooter,
                    conveyor,
                    shooterLimelight,
                    intakeLimelight,
                    drive,
                    lights,
                    driverRotationCommanded),
                new InstantCommand(() -> usingTagHeading = true)));

    // bottom right back button
    driverController
        .povLeft()
        .onTrue(
            new GoHomeSequence(
                    intake, elevator, shooter, conveyor, intakeLimelight, false, true, true)
                .beforeStarting(() -> {
                    lights.setLEDColor(
                        !conveyor.backConveyorBeamBreakSensor.get() ?
                        LightCode.HAS_NOTE :
                        LightCode.OFF);
                })
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
                new FeedPrepScore(
                    elevator,
                    conveyor,
                    intake,
                    shooter,
                    drive,
                    matchState,
                    intakeLimelight,
                    lights)));
    // top left back button
    driverController
        .povUp()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> prepState = PrepState.AMP),
                new AmpPrepScore(elevator, conveyor, intake, shooter, drive, lights),
                new InstantCommand(() -> drive.throttle(0.6))));

    // bottom left back button
    driverController
        .povRight()
        .onTrue(
            new InstantCommand(() -> prepState = PrepState.CLIMB)
                .andThen(
                    new ClimbPrepSequence(
                        intake, elevator, shooter, conveyor, intakeLimelight, lights))
                .andThen(new WaitUntilCommand(() -> elevator.atDesiredPosition()))
                // .andThen(new SetTrapLineupPosition(intakeLimelight,
                // drive).withTimeout(4.0)));
                .andThen(
                    () ->
                        new SequentialCommandGroup(
                                new SetTrapLineupPosition(intakeLimelight, drive),
                                new PIDToPoint(drive),
                                Conveyor.rumbleBriefly(
                                    conveyor)) // rumble after auto drive finishes
                            .raceWith(new WaitUntilCommand(driverJoysticksActive))
                            .schedule())
                .andThen(new InstantCommand(() -> driveFieldRelative = false))
                .andThen(new InstantCommand(() -> drive.throttle(0.3))));

    driverController.back().onTrue(new PartialClimbSequence(intake, elevator, shooter));

    drive.setDefaultCommand(
        new ConditionalCommand(
                new RunCommand(
                    () -> {
                final var translationInputs = JoystickUtil.computeDriveXY(driverController, driveFieldRelative, matchState.isBlue());
                final var rotationInput = JoystickUtil.squareAxis(
                                MathUtil.applyDeadband(-driverController.getRightX(), 0.05));
                drive.rotateOrKeepHeading(
                            translationInputs.getFirst(),
                            translationInputs.getSecond(),
                            rotationInput,
                            driveFieldRelative, // always field relative
                            getCardinalDirectionDegrees());
                    },
                    drive),
                new InstantCommand(),
                () -> matchState.isTeleop())
            .withName("Manual Drive"));
  }

  private void configureOperator() {
    /**
     * conveyor towards shooter -> right trigger -> DONE conveyor outtake -> left trigger -> DONE
     *
     * <p>subwoofer center: speaker prep with set distance -> dpad down -> DONE
     *
     * <p>blue subwoofer amp side (same distance as normal subwoofer shot) -> left dpad -> DONE blue
     * subwoofer source side (same distance as normal subwoofer shot) -> right dpad -> DONE
     *
     * <p>red subwoofer amp side (same distance as normal subwoofer shot) -> right dpad -> DONE red
     * subwoofer source side (same distance as normal subwoofer shot) -> left dpad -> DONE
     *
     * <p>under chain shot (diff headings for blue and red) -> y -> DONE amp shot (diff headings for
     * blue and red) -> b -> DONE podium shot (diff headings for blue and red) -> x -> DONE
     */
    BiFunction<Double, Double, Command> operatorPrepCreator =
        (Double distanceMeters, Double targetHeadingWhenRed) -> {
          return new InstantCommand(() -> shooter.setShooterDistance(distanceMeters))
              .andThen(
                  () -> {
                    final double redDeg = targetHeadingWhenRed;
                    drive.setTargetHeadingDegrees(
                        matchState.isBlue()
                            ? GeometryUtil.flipFieldRotation(Rotation2d.fromDegrees(redDeg))
                                .getDegrees()
                            : redDeg);
                  })
              .andThen(new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)))
              .andThen(new InstantCommand(() -> prepState = PrepState.OPERATOR));
        };

    operatorController.rightTrigger().onTrue(new InstantCommand(() -> conveyor.startRollers(1.0)));
    operatorController.rightTrigger().onFalse(new InstantCommand(() -> conveyor.stopRollers()));
    operatorController
        .leftTrigger()
        .onTrue(
            new InstantCommand(() -> conveyor.startRollers(-1.0))
                .andThen(() -> intake.reverseRollers()));
    operatorController
        .leftTrigger()
        .onFalse(
            new InstantCommand(() -> conveyor.stopRollers()).andThen(() -> intake.stopRollers()));

    operatorController
        .povDown()
        .onTrue(operatorPrepCreator.apply(ShooterCal.SUBWOOFER_SHOT_DISTANCE_METERS, 180.0));
    operatorController
        .povLeft()
        .onTrue(
            operatorPrepCreator.apply(
                ShooterCal.SUBWOOFER_SHOT_DISTANCE_METERS,
                ShooterCal.SUBWOOFER_SHOT_LEFT_RED_DEGREES));

    operatorController
        .povRight()
        .onTrue(
            operatorPrepCreator.apply(
                ShooterCal.SUBWOOFER_SHOT_DISTANCE_METERS,
                ShooterCal.SUBWOOFER_SHOT_RIGHT_RED_DEGREES));

    operatorController
        .b()
        .onTrue(
            operatorPrepCreator.apply(
                ShooterCal.AMP_SHOT_DISTANCE_METERS, ShooterCal.AMP_SHOT_RED_DEGREES));

    operatorController
        .x()
        .onTrue(
            operatorPrepCreator.apply(
                ShooterCal.PODIUM_SHOT_DISTANCE_METERS, ShooterCal.PODIUM_SHOT_RED_DEGREES));

    operatorController
        .y()
        .onTrue(
            operatorPrepCreator.apply(
                ShooterCal.STAGE_SHOT_DISTANCE_METERS, ShooterCal.STAGE_SHOT_RED_DEGREES));

    operatorController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                    () -> intake.rezeroIntakeToPosition(IntakeCal.INTAKE_STOWED_POSITION_DEGREES))
                .ignoringDisable(true));
    operatorController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                    () -> intake.rezeroIntakeToPosition(IntakeCal.INTAKE_DEPLOYED_POSITION_DEGREES))
                .ignoringDisable(true));

    operatorController.start().onTrue(new UnclimbSequence(elevator, shooter, conveyor, lights));
    operatorController.back().onTrue(
        new InstantCommand(() -> elevator.setLeftZeroFromAbsolute()).andThen(
            new InstantCommand(() -> elevator.setRightZeroFromAbsolute())));
    
    // elevator.setDefaultCommand(
    //     new ConditionalCommand(
    //             new RunCommand(
    //                 () -> {
    //             final var translationInputs = JoystickUtil.computeDriveXY(driverController, driveFieldRelative, matchState.isBlue());
    //             final var rotationInput = JoystickUtil.squareAxis(
    //                             MathUtil.applyDeadband(-driverController.getRightX(), 0.05));
    //             drive.rotateOrKeepHeading(
    //                         translationInputs.getFirst(),
    //                         translationInputs.getSecond(),
    //                         rotationInput,
    //                         driveFieldRelative, // always field relative
    //                         getCardinalDirectionDegrees());
    //                 },
    //                 drive),
    //             new InstantCommand(),
    //             () -> matchState.isTeleop())
    //         .withName("Manual Drive"));
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
   * @return true when the currently prepped state is ready to score
   */
  public boolean readyToScoreCheck() {
    switch (prepState) {
      case OFF:
        if (drive.throttleMultiplier != 1.0){
        drive.throttle(1.0);}
        return false;
      case CLIMB:
        if (drive.throttleMultiplier != 0.3){
        drive.throttle(0.3);}
        return intake.nearDeployed() &&
        elevator.atPosition(ElevatorPosition.PRE_CLIMB) &&
        shooter.atDesiredPosition() && PIDToPoint.finishedPid;
      case SPEAKER:
        if (drive.throttleMultiplier != 1.0){
        drive.throttle(1.0);}
        return elevator.atPosition(ElevatorPosition.SLIGHTLY_UP)
            && shooter.atDesiredPosition()
            && shooter.isShooterSpunUp()
            && drive.getDiffCurrentTargetYawDeg() < ShooterCal.ROBOT_HEADING_MARGIN_TO_SHOOT_DEGREES;
      case FEED:
        if (drive.throttleMultiplier != 1.0){
        drive.throttle(1.0);}
        return elevator.atPosition(ElevatorPosition.SLIGHTLY_UP)
            && shooter.atDesiredPosition()
            && shooter.isShooterSpunUp();
      case AMP:
        if (drive.throttleMultiplier != 0.6){
        drive.throttle(0.6);}
        return elevator.atPosition(ElevatorPosition.SCORE_AMP);
      case OPERATOR:
        if (drive.throttleMultiplier != 1.0){
        drive.throttle(1.0);}
        return shooter.isShooterSpunUp()
            && shooter.atDesiredPosition()
            && drive.getDiffCurrentTargetYawDeg()
                < ShooterCal.ROBOT_HEADING_MARGIN_TO_SHOOT_DEGREES;
    }
    return false;
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

  public String getAutonomousName() {
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
    builder.addBooleanProperty("Using heading from tag", () -> usingTagHeading, null);
    builder.addBooleanProperty("ready to score (leds)", () -> readyToScoreCheck(), null);
  }
}

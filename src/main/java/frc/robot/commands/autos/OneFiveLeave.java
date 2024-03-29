package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.RotateToSpeaker;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.commands.autos.components.ScoreThisNote;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import frc.robot.utils.MatchStateUtil;

/** score the preloaded note into the speaker, then get another note, return to home state */
public class OneFiveLeave extends SequentialCommandGroup {
  public OneFiveLeave(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      DriveSubsystem drive,
      MatchStateUtil matchState,
      ShooterLimelight shooterLimelight,
      IntakeLimelight intakeLimelight,
      Lights lights) {
    final double X_DISTANCE_TO_NOTE_METERS = 1.0; // guesstimate
    final double NO_Y_DISTANCE = 0.0;
    final boolean FIELD_RELATIVE = true;

    addRequirements(intake, elevator, shooter, conveyor, drive);
    addCommands(
        new RotateToSpeaker(drive, shooterLimelight),
        drive.turnInPlace(1.0),
        drive.stopDrivingCommand(),
        new PrintCommand("OneFiveLeave - Done rotating"),
        new ScoreThisNote(
            intake, elevator, shooter, conveyor, shooterLimelight, intakeLimelight, drive, lights),
        new PrintCommand("OneFiveLeave - Done scoring"),
        new ParallelCommandGroup(
            new PrintCommand("OneFiveLeave - Intaking"),
            new IntakeSequence(intake, elevator, conveyor, shooter, lights)
                .withTimeout(IntakeConstants.INTAKING_TIMEOUT_SEC),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      drive.setForwardTargetHeading();
                    }),
                drive.turnInPlace(1.0),
                new DriveDistance(
                    drive,
                    0.2,
                    X_DISTANCE_TO_NOTE_METERS,
                    NO_Y_DISTANCE,
                    FIELD_RELATIVE,
                    matchState))));
  }
}

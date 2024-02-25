package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.MatchState;
import frc.robot.commands.RotateToSpeaker;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.commands.autos.components.ScoreThisNote;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;

/**
 * score the preloaded note into the speaker, then get another note and score it in the speaker,
 * return to home state
 */
public class ScoreTwoNotes extends SequentialCommandGroup {
  private Rotation2d initialYaw = new Rotation2d();

  public ScoreTwoNotes(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      DriveSubsystem drive,
      MatchState matchState,
      ShooterLimelight limelight) {

    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new InstantCommand(() -> initialYaw = drive.getPose().getRotation()),
        new OneFiveLeave(intake, elevator, shooter, conveyor, drive, matchState, limelight),
        new InstantCommand(
            () -> {
              final Rotation2d currentYaw = drive.getPose().getRotation();
              final Rotation2d differenceYaw = currentYaw.minus(initialYaw);
              final Rotation2d halfwayYaw = initialYaw.plus(differenceYaw.div(2.0));
              drive.setTargetHeadingDegrees(halfwayYaw.getDegrees());
            }),
        drive.turnInPlace(0.5),
        new RotateToSpeaker(drive, limelight),
        drive.turnInPlace(0.5),
        new ScoreThisNote(
            intake, elevator, shooter, conveyor, limelight // goes home at the end of ScoreThisNote
            ),
        new InstantCommand(drive::setForwardTargetHeading),
        drive.turnInPlace(0.5),
        new DriveDistance(drive, 0.2, 0.2, 0.0, true, matchState));
  }
}

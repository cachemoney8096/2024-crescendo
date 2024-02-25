package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer.MatchState;
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
        new OneFiveLeave(intake, elevator, shooter, conveyor, drive, matchState, limelight),
        new ScoreThisNote(
            intake, elevator, shooter, conveyor, limelight) // goes home at the end of ScoreThisNote
        );
  }
}

package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.SpeakerPrepScoreSequence;
import frc.robot.commands.SpeakerShootSequence;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * score the already loaded (intake-ed/intook?) note into the speaker and return to the home state
 */
public class ScoreThisNote extends SequentialCommandGroup {
  public ScoreThisNote(Intake intake, Elevator elevator, Shooter shooter, Conveyor conveyor) {
    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new SpeakerPrepScoreSequence(intake, elevator, shooter),
        new SpeakerShootSequence(conveyor, shooter),
        new GoHomeSequence(intake, elevator, shooter, false));
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Scores a note in the amp. At the moment, this only calls the scoreTrapOrAmp command in the
 * conveyor. However, we should add functionality in the future to "smartly" go home, in which case
 * we will likely want a separate command group.
 */
public class AmpScore extends SequentialCommandGroup {
  public AmpScore(Conveyor conveyor) {
    addRequirements(conveyor);
    addCommands(
        Conveyor.scoreTrapOrAmp(conveyor)
        // TODO "smartly" go home (e.g. go home once we've driven away from the wall)
        );
  }

  /** runs AmpScore but adds the layer that if the robot is interrupted, then everything is brought back to the prep state */
  public static Command interruptibleAmpScore(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    return new AmpScore(conveyor)
        .finallyDo(
            (boolean interrupted) -> {
              AmpPrepScore.interruptibleAmpPrepScore(intake, elevator, conveyor, shooter);
            });
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

/**
 * Scores a note in the amp. At the moment, this only calls the scoreTrapOrAmp command in the
 * conveyor. However, we should add functionality in the future to "smartly" go home, in which case
 * we will likely want a separate command group.
 */
public class AmpScore extends SequentialCommandGroup {
  public AmpScore(Conveyor conveyor, Intake intake, Elevator elevator) {
    addRequirements(conveyor, intake, elevator);
    addCommands(
      new WaitUntilCommand(() -> {
              return intake.clearOfConveyorZone() && elevator.atDesiredPosition();
            }),
        Conveyor.scoreTrapOrAmp(conveyor)
        // TODO "smartly" go home (e.g. go home once we've driven away from the wall)
        );
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class AmpPrepScore extends SequentialCommandGroup {
  /** Creates a new AmpPreScore. */
  public AmpPrepScore(Elevator elevator, Conveyor conveyer, Intake intake) {
    addRequirements(conveyer, elevator, intake);

    SequentialCommandGroup moveWhenNotSafe =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> intake.setDesiredIntakePosition(IntakePosition.CLEAR_OF_CONVEYOR)),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP)),
            new WaitUntilCommand(elevator::elevatorAboveInterferenceZone));

    addCommands(
        new ConditionalCommand(
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP)),
            moveWhenNotSafe,
            () -> elevator.elevatorAboveInterferenceZone()),
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
        new WaitUntilCommand(
            () -> {
              return intake.atDesiredIntakePosition() && elevator.atDesiredPosition();
            }));
  }
}

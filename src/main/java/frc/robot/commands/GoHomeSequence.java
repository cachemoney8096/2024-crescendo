package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

/**
 * brings the elevator and intake to their respective home positions in a "smart" way so they do not
 * end up attacking each other accidentally
 */
public class GoHomeSequence extends SequentialCommandGroup {
  public GoHomeSequence(Intake intake, Elevator elevator) {
    final SequentialCommandGroup goHomeWhenSafe =
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME)),
            new WaitUntilCommand(
                () -> {
                  return intake.atDesiredIntakePosition() && elevator.atDesiredPosition();
                }));

    final SequentialCommandGroup goHomeWhenNotSafe =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> intake.setDesiredIntakePosition(IntakePosition.CLEAR_OF_CONVEYOR)),
            new WaitUntilCommand(intake::atDesiredIntakePosition),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME)),
            new WaitUntilCommand(elevator::elevatorBelowInterferenceZone),
            new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
            new WaitUntilCommand(
                () -> {
                  return intake.atDesiredIntakePosition() && elevator.atDesiredPosition();
                }));

    addRequirements(intake, elevator);
    addCommands(
        new ConditionalCommand(
            goHomeWhenSafe, goHomeWhenNotSafe, () -> elevator.elevatorBelowInterferenceZone()));
  }
}

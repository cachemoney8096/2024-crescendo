package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class GoHomeSequence extends SequentialCommandGroup {
  public GoHomeSequence(Intake intake, Elevator elevator) {
    addRequirements(intake, elevator);
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME)),
                new WaitUntilCommand(
                    () -> {
                      return intake.atDesiredIntakePosition() && elevator.atDesiredPosition();
                    })),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> intake.setDesiredIntakePosition(IntakePosition.CLEAR_OF_CONVEYOR)),
                new WaitUntilCommand(intake::atDesiredIntakePosition),
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME)),
                new WaitUntilCommand(elevator::elevatorBelowInterferenceThreshold),
                new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
                new WaitUntilCommand(
                    () -> {
                      return intake.atDesiredIntakePosition() && elevator.atDesiredPosition();
                    })),
            () -> elevator.elevatorBelowInterferenceThreshold()));
  }
}

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

public class IntakeSequence extends SequentialCommandGroup {
  public IntakeSequence(Intake intake, Elevator elevator, Conveyor conveyor) {
    addRequirements(intake, elevator);

    addCommands(
        new ConditionalCommand(
            new InstantCommand(),
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> intake.setDesiredIntakePosition(IntakePosition.CLEAR_OF_CONVEYOR)),
                new WaitUntilCommand(intake::atDesiredIntakePosition),
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME)),
                new WaitUntilCommand(elevator::elevatorBelowInterferenceThreshold)),
            elevator::atDesiredPosition),
        new InstantCommand(
            () -> {
              intake.setDesiredIntakePosition(IntakePosition.DEPLOYED);
            },
            intake),
        new WaitUntilCommand(intake::atDesiredIntakePosition),
        new InstantCommand(intake::startRollers),
        Conveyor.receive(conveyor),
        new InstantCommand(
            () -> {
              intake.stopRollers();
            },
            intake),
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)));
  }
}

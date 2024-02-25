package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

/* moves elevator out of the way before deploying the intake if the elevator is at the top of the interference zone */
public class SafeDeploy extends SequentialCommandGroup {
  public SafeDeploy(Intake intake, Elevator elevator) {
    addRequirements(intake, elevator);
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP, true)),
                new WaitUntilCommand(elevator::elevatorAboveIntakeInterferenceZone)),
            new InstantCommand(),
            elevator::eleavtorAtTopOfIntakeInterferenceZone),
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)));
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class IntakeSequence extends SequentialCommandGroup{
  public IntakeSequence(Intake intake, Elevator elevator, Conveyor conveyor) {
    addRequirements(intake, elevator);

    addCommands(
      new ConditionalCommand(new InstantCommand(), new InstantCommand(() -> {elevator.setDesiredPosition(ElevatorPosition.HOME);}, elevator), elevator::atDesiredPosition),

      new WaitUntilCommand(elevator::atDesiredPosition),

      new InstantCommand( 
      () -> {
        intake.moveToPos(IntakePosition.DEPLOYED);}, intake),

      new WaitUntilCommand(intake::atDesiredIntakePosition),

      new InstantCommand(intake::startRollers),

      new WaitUntilCommand(conveyor::recieve),

      new InstantCommand( 
      () -> {
        intake.stopRollers();}, intake)
    );

  }
}
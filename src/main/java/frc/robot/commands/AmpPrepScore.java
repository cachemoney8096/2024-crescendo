// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpPrepScore extends SequentialCommandGroup {
  /** Creates a new AmpPreScore. */
  public AmpPrepScore(Elevator elevator, Conveyor conveyer, Intake intake) {
    addRequirements(conveyer, elevator, intake);

    SequentialCommandGroup moveWhenSafe =
        new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP)),
            new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
            new WaitUntilCommand(
                () -> {
                  return elevator.atDesiredPosition() && intake.atDesiredIntakePosition();
                }));

    SequentialCommandGroup moveWhenNotSafe =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> intake.setDesiredIntakePosition(IntakePosition.CLEAR_OF_CONVEYOR)),
            new WaitUntilCommand(intake::atDesiredIntakePosition),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME)),
            new WaitUntilCommand(elevator::elevatorAboveInterferenceZone),
            new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
            new WaitUntilCommand(
                () -> {
                  return intake.atDesiredIntakePosition() && elevator.atDesiredPosition();
                }));

    addCommands(
        new WaitUntilCommand(
            () -> conveyer.currentNotePosition == Conveyor.ConveyorPosition.HOLDING_NOTE),
        new ConditionalCommand(
            moveWhenSafe, moveWhenNotSafe, () -> elevator.elevatorOutsideInterferenceZone()));
  }
}

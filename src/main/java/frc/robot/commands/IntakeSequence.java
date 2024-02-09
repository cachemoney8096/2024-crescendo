package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Puts the intake into the deployed position then gets elevator to home position Checks that
 * everything is in position to intake then starts rollers Intakes until the conveyor recieves the
 * game piece Stops and stowes intake
 */
public class IntakeSequence extends SequentialCommandGroup {
  public IntakeSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    addRequirements(intake, elevator, conveyor);

    addCommands(
        new InstantCommand(
            () -> {
              intake.setDesiredIntakePosition(IntakePosition.DEPLOYED);
            },
            intake),
        new ConditionalCommand(
            new InstantCommand(),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            elevator::elevatorAboveInterferenceZone),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME)),
        new WaitUntilCommand(elevator::atDesiredPosition),
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

  /** runs IntakeSequence but adds the layer that if the robot is interrupted, then everything is brought back to the home state */
  public static Command interruptibleIntakeSequnce(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    return new IntakeSequence(intake, elevator, conveyor, shooter)
        .finallyDo(
            (boolean interrupted) -> {
              new StopMostThings(intake, elevator, conveyor, shooter);
            });
  }
}

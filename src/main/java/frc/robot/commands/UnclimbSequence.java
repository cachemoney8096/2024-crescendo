package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

public class UnclimbSequence extends SequentialCommandGroup {
  public UnclimbSequence(Elevator elevator, Shooter shooter, Conveyor conveyor) {
    addRequirements(elevator, shooter, conveyor);
    addCommands(
        Conveyor.stop(conveyor),
        new InstantCommand(
            () -> {
              elevator.setDesiredPosition(ElevatorPosition.HOME, false);
            }),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new InstantCommand(
            () -> {
              shooter.setShooterMode(ShooterMode.PRELATCH);
            }),
        new WaitUntilCommand(shooter::atDesiredPosition),
        new InstantCommand(
            () -> {
              elevator.setDesiredPosition(ElevatorPosition.SCORE_TRAP, false);
            }));
  }
}

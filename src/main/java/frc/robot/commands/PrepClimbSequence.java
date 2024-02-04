package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

public class PrepClimbSequence extends SequentialCommandGroup {
  public PrepClimbSequence(Intake intake, Elevator elevator, Shooter shooter) {
    addRequirements(intake, elevator, shooter);
    addCommands(
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)),
        new WaitUntilCommand(intake::atDesiredIntakePosition),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.PRE_CLIMB)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.PRELATCH)),
        new WaitUntilCommand(shooter::atDesiredPosition));
  }
}

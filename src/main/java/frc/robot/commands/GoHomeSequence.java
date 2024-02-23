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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * Brings the elevator, intake, and shooter to their respective home positions in a "smart" way so
 * they do not end up attacking each other accidentally. Ends before everything is fully home.
 */
public class GoHomeSequence extends SequentialCommandGroup {
  public GoHomeSequence(
      Intake intake, Elevator elevator, Shooter shooter, Conveyor conveyor, boolean spinUpShooter) {
    final ShooterMode desiredShooterMode = spinUpShooter ? ShooterMode.SPIN_UP : ShooterMode.IDLE;
    final SequentialCommandGroup goHomeWhenSafe =
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
            new InstantCommand(() -> shooter.setShooterMode(desiredShooterMode)),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)));

    final SequentialCommandGroup goHomeWhenNotSafe =
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)),
            new InstantCommand(() -> shooter.setShooterMode(desiredShooterMode)),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            new WaitUntilCommand(shooter::clearOfConveyorZone),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)),
            new WaitUntilCommand(elevator::elevatorBelowInterferenceZone),
            new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)));

    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new InstantCommand(conveyor::stopRollers),
        new InstantCommand(intake::stopRollers),
        new ConditionalCommand(
            goHomeWhenSafe, goHomeWhenNotSafe, () -> elevator.elevatorBelowInterferenceZone()));
  }
}

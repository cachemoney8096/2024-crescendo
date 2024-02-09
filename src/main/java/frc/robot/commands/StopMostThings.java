package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/** returns the robot to a home state without anything moving or shooting
 * should run when the HOME button is pressed
 */
public class StopMostThings extends SequentialCommandGroup {
  public StopMostThings(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    addRequirements(intake, elevator, conveyor, shooter);
    addCommands(
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.IDLE)),
        Conveyor.stop(conveyor),
        new InstantCommand(intake::stopRollers),
        new GoHomeSequence(intake, elevator, conveyor, shooter));
  }
}

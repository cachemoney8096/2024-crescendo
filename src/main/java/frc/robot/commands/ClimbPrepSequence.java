package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * Gets the robot ready to climb by moving the intake, elevator, and shooter to their appropriate
 * positions. Does not wait until everything is at their desired positions!
 */
public class ClimbPrepSequence extends SequentialCommandGroup {
  public ClimbPrepSequence(Intake intake, Elevator elevator, Shooter shooter) {
    addRequirements(intake, elevator, shooter);
    SequentialCommandGroup waitTillSafe =
        new SequentialCommandGroup(
            new WaitUntilCommand(shooter::clearOfConveyorZone),
            new WaitUntilCommand(intake::clearOfConveyorZone));

    addCommands(
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.PRELATCH)),
        new ConditionalCommand(
            new InstantCommand(), waitTillSafe, elevator::elevatorAboveInterferenceZone),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.PRE_CLIMB)));
  }
}
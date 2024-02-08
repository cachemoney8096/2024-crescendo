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
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * gets the robot ready to climb by moving the intake, elevator, and shooter to their appropriate
 * positions
 */
public class PrepClimbSequence extends SequentialCommandGroup {
  public PrepClimbSequence(Intake intake, Elevator elevator, Shooter shooter) {
    addRequirements(intake, elevator, shooter);
    addCommands(
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)),
        new ConditionalCommand(
            new InstantCommand(),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            elevator::elevatorAboveInterferenceZone),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.PRE_CLIMB)),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.PRELATCH)));
  }

  /** runs PrepClimbSequence but adds the layer that if the robot is interrupted, then everything is brought back to the home state */
  public static Command interruptiblePrepClimbSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    return new PrepClimbSequence(intake, elevator, shooter)
        .finallyDo(
            (boolean interrupted) -> {
              new StopMostThings(intake, elevator, conveyor, shooter);
            });
  }
}

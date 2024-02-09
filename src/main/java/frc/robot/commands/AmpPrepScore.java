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
 * Prepares to score in the amp. This command group safely moves the intake and elevator to the
 * score amp position.
 */
public class AmpPrepScore extends SequentialCommandGroup {
  /** Creates a new AmpPreScore. */
  public AmpPrepScore(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    addRequirements(elevator, intake);

    SequentialCommandGroup moveWhenNotSafe =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> intake.setDesiredIntakePosition(IntakePosition.CLEAR_OF_CONVEYOR)),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP)),
            new WaitUntilCommand(elevator::elevatorAboveInterferenceZone));

    addCommands(
        new ConditionalCommand(
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP)),
            moveWhenNotSafe,
            () -> elevator.elevatorAboveInterferenceZone()),
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
        new WaitUntilCommand(
            () -> {
              return intake.atDesiredIntakePosition() && elevator.atDesiredPosition();
            }));
  }

  /**
   * runs AmpPrepScore but adds the layer that if the robot is interrupted, then everything is
   * brought to the home state
   */
  public static Command interruptibleAmpPrepScore(
      Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    return new AmpPrepScore(intake, elevator, conveyor, shooter)
        .finallyDo(
            (boolean interrupted) -> {
              new StopMostThings(intake, elevator, conveyor, shooter);
            });
  }
}

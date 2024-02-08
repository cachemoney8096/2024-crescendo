package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * switches the elevator to use the climbing controller, then has the robot climb and then score in
 * the trap
 */
public class ClimbSequence extends SequentialCommandGroup {
  public ClimbSequence(Intake intake, Elevator elevator, Shooter shooter, Conveyor conveyor) {
    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new WaitUntilCommand(
            () -> {
              return intake.atDesiredIntakePosition()
                  && elevator.atDesiredPosition()
                  && shooter.atDesiredPosition();
            }),
        new InstantCommand(() -> elevator.setControlParams(false)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.POST_CLIMB)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.LATCH)),
        new WaitUntilCommand(shooter::atDesiredPosition),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_TRAP)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        Conveyor.scoreTrapOrAmp(conveyor));
  }

  /** runs ClimbSequence but adds the layer that if the robot is interrupted, then everything is brought to the prep state */
  public static Command interruptibleClimbSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    return new ClimbSequence(intake, elevator, shooter, conveyor)
        .finallyDo(
            (boolean interrupted) -> {
              PrepClimbSequence.interruptiblePrepClimbSequence(intake, elevator, conveyor, shooter);
            });
  }

}

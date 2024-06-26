package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * Switches the elevator to use the climbing controller, then has the robot climb and then score in
 * the trap. Assumes that the robot is aleady lined up to the trap/chain.
 */
public class ClimbSequence extends SequentialCommandGroup {
  public ClimbSequence(Intake intake, Elevator elevator, Shooter shooter, Conveyor conveyor) {
    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        // new WaitUntilCommand(
        //     () -> {
        //       return intake.nearDeployed()
        //           && elevator.atDesiredPosition()
        //           && shooter.atDesiredPosition();
        //     }),
        new InstantCommand(() -> elevator.setControlParams(false)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, false)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new WaitCommand(0.25),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.LATCH)),
        new WaitUntilCommand(shooter::atDesiredPosition),
        new WaitCommand(0.25),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_TRAP, true)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new WaitCommand(0.5),
        Conveyor.scoreTrapOrAmp(conveyor, false));
  }
}

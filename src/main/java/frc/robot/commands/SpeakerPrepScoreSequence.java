package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * gets the robot ready to shoot a ring into the speaker. gets intake and elevator into position,
 * spins up the shooter
 */
public class SpeakerPrepScoreSequence extends SequentialCommandGroup {

  public SpeakerPrepScoreSequence(Intake intake, Elevator elevator, Shooter shooter) {
    addRequirements(intake, elevator, shooter);
    addCommands(
        new InstantCommand(
            () -> shooter.setShooterDistance(shooter.getDistanceFromDistanceGetter())),
        new GoHomeSequence(intake, elevator, shooter, true),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)));
  }
}

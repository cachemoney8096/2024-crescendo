package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * gets the robot ready to shoot a ring into the speaker. gets intake and elevator into position,
 * spins up the shooter
 */
public class SpeakerPrepScoreSequence extends SequentialCommandGroup {
  /** this is a specified distance from the speaker each time until we do limelight stuff */
  public final double SPEAKER_SHOOTER_DISTANCE_METERS = 5.0;

  public SpeakerPrepScoreSequence(
      Intake intake, Elevator elevator, Shooter shooter, Conveyor conveyor) {
    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new InstantCommand(() -> shooter.setShooterDistance(SPEAKER_SHOOTER_DISTANCE_METERS)),
        new GoHomeSequence(intake, elevator, shooter, conveyor, true),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)));
  }
}

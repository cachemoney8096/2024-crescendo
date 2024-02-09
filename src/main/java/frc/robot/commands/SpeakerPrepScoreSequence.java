package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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

  public SpeakerPrepScoreSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    addRequirements(intake, elevator, shooter);
    addCommands(
        new InstantCommand(() -> shooter.setShooterDistance(SPEAKER_SHOOTER_DISTANCE_METERS)),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new GoHomeSequence(intake, elevator, conveyor, shooter));
  }

  /** runs SpeakerPrepScoreSequence but adds the layer that if the robot is interrupted, then everything is brought back to the home state */
  public static Command interruptibleSpeakerPrepScoreSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    return new SpeakerPrepScoreSequence(intake, elevator, conveyor, shooter)
        .finallyDo(
            (boolean interrupted) -> {
              new StopMostThings(intake, elevator, conveyor, shooter);
            });
  }
}

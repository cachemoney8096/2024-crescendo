package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

public class SpeakerPrepScoreSequence extends SequentialCommandGroup {
  /**
   * this is a specified distance from the speaker each time until we do limelight
   * stuff
   */
  public final double SPEAKER_SHOOTER_DISTANCE_METERS = 5.0;

  public SpeakerPrepScoreSequence(Intake intake, Elevator elevator, Shooter shooter) {
    addRequirements(intake, elevator, shooter);
    addCommands(
        new InstantCommand(() -> shooter.setShooterDistance(SPEAKER_SHOOTER_DISTANCE_METERS)),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new GoHomeSequence(intake, elevator));
  }
}

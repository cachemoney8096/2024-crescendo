package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.conveyer.Conveyer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

public class SpeakerPrepScoreSequence extends SequentialCommandGroup {
   /** this is a specified distance from the speaker each time until we do limelight stuff */
  public final double SPEAKER_SHOOTER_DISTANCE = Constants.PLACEHOLDER_DOUBLE;

  public SpeakerPrepScoreSequence(Intake intake, Elevator elevator, Conveyer conveyor, Shooter shooter) {
    addRequirements(intake, elevator, conveyor, shooter);
    addCommands(
        new GoHomeSequence(intake, elevator, conveyor),

        new InstantCommand(() -> shooter.setShooterDistance(SPEAKER_SHOOTER_DISTANCE)),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new WaitUntilCommand(() -> {
          return shooter.atDesiredPosition() && shooter.isShooterSpunUp();
        }));
  }
}

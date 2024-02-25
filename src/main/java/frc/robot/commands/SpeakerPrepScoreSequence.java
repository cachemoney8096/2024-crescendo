package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import java.util.Optional;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * gets the robot ready to shoot a ring into the speaker. gets intake and elevator into position,
 * spins up the shooter
 */
public class SpeakerPrepScoreSequence extends SequentialCommandGroup {
  ShooterLimelight limelight;
  
  Optional<Pair<Rotation2d, Double>> tagDetection;
  double distanceFromSpeakerMeters = 0.0;

  public SpeakerPrepScoreSequence(
      Intake intake, Elevator elevator, Shooter shooter, Conveyor conveyor, ShooterLimelight limelight) {
        this.limelight = limelight;

    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new WaitUntilCommand(() -> {
          tagDetection = limelight.checkForTag();
          return tagDetection.isPresent();
        }),
        new InstantCommand(() -> distanceFromSpeakerMeters = tagDetection.get().getSecond()),
        new InstantCommand(() -> shooter.setShooterDistance(distanceFromSpeakerMeters)),
        new GoHomeSequence(intake, elevator, shooter, conveyor, true),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new InstantCommand(() -> conveyor.startBackRollers(1.0)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SPEAKER_PREP, true)));
        // Conveyor.backUpNote(conveyor));
  }
}

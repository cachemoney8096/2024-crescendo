package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterCal;
import java.util.Optional;

/**
 * gets the robot ready to shoot a ring into the speaker. gets intake and elevator into position,
 * spins up the shooter
 */
public class SpeakerPrepScoreAutoPreload extends SequentialCommandGroup {

  Optional<Pair<Rotation2d, Double>> tagDetection = Optional.empty();
  double distanceFromSpeakerMeters = 0.0;

  public SpeakerPrepScoreAutoPreload(
      Intake intake, Elevator elevator, Shooter shooter, Conveyor conveyor) {

    // Drive is not a requirement!!
    addRequirements(intake, elevator, shooter, conveyor);

    addCommands(
        new GoHomeSequence(intake, elevator, shooter, conveyor, true, false),
        // new InstantCommand(() -> conveyor.startBackRollers(1.0)),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new InstantCommand(() -> shooter.setShooterDistance(ShooterCal.AUTO_PRELOAD_DISTANCE_M)));
  }
}

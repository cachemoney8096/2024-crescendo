package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import java.util.Optional;

/**
 * gets the robot ready to shoot a ring into the speaker. gets intake and elevator into position,
 * spins up the shooter
 */
public class SpeakerPrepScoreAuto extends SequentialCommandGroup {

  Optional<Pair<Rotation2d, Double>> tagDetection = Optional.empty();

  public SpeakerPrepScoreAuto(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      double shooterDistanceMeters) {

    // Drive is not a requirement!!
    addRequirements(intake, elevator, shooter, conveyor);

    addCommands(
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new InstantCommand(() -> shooter.setShooterDistance(shooterDistanceMeters)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SLIGHTLY_UP, true)),
        new WaitUntilCommand(() -> elevator.atDesiredPosition()));
  }
}

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer.MatchState;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveCal;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Scores a note in the amp. At the moment, this only calls the scoreTrapOrAmp command in the
 * conveyor. However, we should add functionality in the future to "smartly" go home, in which case
 * we will likely want a separate command group.
 */
public class AmpScore extends SequentialCommandGroup {
  public AmpScore(
      Conveyor conveyor,
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      DriveSubsystem drive,
      MatchState matchState) {
    final double NO_X_MOVEMENT = 0.0;
    final double Y_MOVE_18_INCHES_AWAY_FROM_AMP =
        -1.0 * Units.inchesToMeters(18.0); // TODO check if this actually should be negative
    final boolean FIELD_RELATIVE = true;
    addRequirements(conveyor, intake, elevator, shooter, drive);
    addCommands(
        new WaitUntilCommand(
            () -> {
              return intake.clearOfConveyorZone() && elevator.atDesiredPosition();
            }),
        Conveyor.scoreTrapOrAmp(conveyor),
        new DriveDistance(
            drive,
            DriveCal.MEDIUM_LINEAR_SPEED_METERS_PER_SEC,
            NO_X_MOVEMENT,
            Y_MOVE_18_INCHES_AWAY_FROM_AMP,
            FIELD_RELATIVE,
            matchState),
        new GoHomeSequence(intake, elevator, shooter, false));
  }
}

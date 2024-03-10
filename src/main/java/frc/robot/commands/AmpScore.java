package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
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
  Pose2d initialPose;

  public AmpScore(
      DriveSubsystem drive, Conveyor conveyor, Intake intake, Shooter shooter, Elevator elevator) {
    // Drive not a requirement
    addRequirements(conveyor, intake, elevator, shooter);

    addCommands(
        new InstantCommand(() -> initialPose = drive.getPose()),
        new WaitUntilCommand(
            () -> {
              return elevator.atDesiredPosition();
            }),
        Conveyor.scoreTrapOrAmp(conveyor),
        new WaitUntilCommand(
            () ->
                Math.abs(
                        drive.getPose().getTranslation().minus(initialPose.getTranslation()).getY())
                    > 0.1),
        new GoHomeSequence(intake, elevator, shooter, conveyor, false, true, true));
  }
}

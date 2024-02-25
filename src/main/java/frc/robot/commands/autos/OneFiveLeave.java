package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.MatchState;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.autos.components.DriveDistance;
import frc.robot.commands.autos.components.ScoreThisNote;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;

/** score the preloaded note into the speaker, then get another note, return to home state */
public class OneFiveLeave extends SequentialCommandGroup {
  public OneFiveLeave(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      DriveSubsystem drive,
      MatchState matchState) {
    final double X_DISTANCE_TO_NOTE_METERS = 1.0; // guesstimate
    final double NO_Y_DISTANCE = 0.0;
    final boolean FIELD_RELATIVE = true;
    final double WAIT_BEFORE_DRIVE_SEC = 1.0;

    addRequirements(intake, elevator, shooter, conveyor, drive);
    addCommands(
        new ScoreThisNote(intake, elevator, shooter, conveyor),
        new ParallelCommandGroup(
            new IntakeSequence(intake, elevator, conveyor, shooter, new InstantCommand())
                .withTimeout(IntakeConstants.INTAKING_TIMEOUT_SEC),
            new SequentialCommandGroup(
                new WaitCommand(WAIT_BEFORE_DRIVE_SEC),
                new DriveDistance(
                    drive,
                    DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                    X_DISTANCE_TO_NOTE_METERS,
                    NO_Y_DISTANCE,
                    FIELD_RELATIVE,
                    matchState))));
  }
}

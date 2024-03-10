package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.SpeakerPrepScoreSequence;
import frc.robot.commands.SpeakerShootSequence;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;

/**
 * score the already loaded (intake-ed/intook?) note into the speaker and return to the home state
 */
public class ScoreThisNote extends SequentialCommandGroup {
  public ScoreThisNote(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      ShooterLimelight limelight,
      DriveSubsystem drive) {
    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new SpeakerPrepScoreSequence(
                intake,
                elevator,
                shooter,
                conveyor,
                limelight,
                drive,
                () -> {
                  return false;
                })
            .withTimeout(5.0),
        new PrintCommand("ScoreThisNote - done prep"),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SLIGHTLY_UP, true)),
        new WaitUntilCommand(() -> elevator.atDesiredPosition()),
        new SpeakerShootSequence(conveyor, shooter, elevator, drive),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)),
        new PrintCommand("ScoreThisNote - done score"),
        new GoHomeSequence(intake, elevator, shooter, conveyor, false, false, false),
        new PrintCommand("ScoreThisNote - done home"));
  }
}

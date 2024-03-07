package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;

public class ScoreFourFromCenterLine extends SequentialCommandGroup {
  public ScoreFourFromCenterLine(
      boolean red,
      DriveSubsystem drive,
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      ShooterLimelight limelight) {

    addRequirements(drive, intake, elevator, shooter, conveyor, limelight);

    // register commands for PathPlanner
    // NamedCommands.registerCommand("SPEAKER PREP PRELOAD", new SpeakerPrepScoreAutoPreload(intake,
    // elevator, shooter, conveyor));
    // NamedCommands.registerCommand("SPEAKER SCORE PRELOAD", new SpeakerShootSequence(conveyor,
    // shooter, elevator));
    // NamedCommands.registerCommand("INTAKE DEPLOYED", new
    // InstantCommand(()->intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)));
    // NamedCommands.registerCommand("RUN INTAKE 1", new IntakeSequence(intake, elevator, conveyor,
    // shooter));
    // NamedCommands.registerCommand("SPEAKER PREP 1", new SpeakerPrepScoreAuto(intake, elevator,
    // shooter, conveyor));
    // NamedCommands.registerCommand("SPEAKER SCORE 1", new SpeakerShootSequence(conveyor, shooter,
    // elevator));
    // NamedCommands.registerCommand("RUN INTAKE 2", new IntakeSequence(intake, elevator, conveyor,
    // shooter));
    // NamedCommands.registerCommand("SPEAKER PREP 2", new SpeakerPrepScoreAuto(intake, elevator,
    // shooter, conveyor));
    // NamedCommands.registerCommand("SPEAKER SCORE 2", new SpeakerShootSequence(conveyor, shooter,
    // elevator));
    // NamedCommands.registerCommand("RUN INTAKE 3", new IntakeSequence(intake, elevator, conveyor,
    // shooter));
    // NamedCommands.registerCommand("SPEAKER PREP 3", new SpeakerPrepScoreAuto(intake, elevator,
    // shooter, conveyor));
    // NamedCommands.registerCommand("SPEAKER SCORE 3", new SpeakerShootSequence(conveyor, shooter,
    // elevator));

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(new PathPlannerAuto("4 NOTE - CENTER LINE - SOURCE - AUTO"));
  }
}

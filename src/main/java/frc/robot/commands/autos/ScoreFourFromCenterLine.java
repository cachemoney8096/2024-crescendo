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
      DriveSubsystem drive,
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      ShooterLimelight limelight) {

    addRequirements(drive, intake, elevator, shooter, conveyor, limelight);

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(new PathPlannerAuto("4 NOTE - CENTER LINE - SOURCE - AUTO"));
  }
}

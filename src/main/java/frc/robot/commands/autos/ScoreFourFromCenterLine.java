package frc.robot.commands.autos;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;

public class ScoreFourFromCenterLine extends SequentialCommandGroup {
  private final String pathName = "Copy of 4 NOTE - CENTER LINE - SOURCE";
  private PathPlannerPath autoPath =
      PathPlannerPath.fromPathFile(pathName);

  public ScoreFourFromCenterLine(boolean red, 
      DriveSubsystem drive,
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      ShooterLimelight limelight) {

    addRequirements(drive, intake, elevator, shooter, conveyor, limelight);
    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        drive.followTrajectoryCommand(autoPath, true));
      }
}

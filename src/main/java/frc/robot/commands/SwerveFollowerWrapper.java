package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;

public class SwerveFollowerWrapper extends Command {
  public Command swerveFollowerCmd;
  private DriveSubsystem drive;
  boolean noTrajectory = true;

  public SwerveFollowerWrapper(DriveSubsystem driveSubsystem) {
    drive = driveSubsystem;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Optional<PathPlannerPath> maybeTrajectory = drive.poseToPath();
    if (!maybeTrajectory.isPresent()) {
      return;
    }
    noTrajectory = false;
    swerveFollowerCmd =
        drive.followTrajectoryCommand(maybeTrajectory.get(), false).withTimeout(3.0);
    swerveFollowerCmd.initialize();
  }

  @Override
  public void execute() {
    if (!noTrajectory) {
      swerveFollowerCmd.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!noTrajectory) {
      swerveFollowerCmd.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    if (!noTrajectory) {
      return swerveFollowerCmd.isFinished();
    } else {
      return true;
    }
  }
}

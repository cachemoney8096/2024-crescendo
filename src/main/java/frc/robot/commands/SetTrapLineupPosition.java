package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;

public class SetTrapLineupPosition extends Command{
  private DriveSubsystem drive;
  private IntakeLimelight tagLimelight;
  private boolean targetLocked = false;

  public SetTrapLineupPosition(
      IntakeLimelight limelight, DriveSubsystem driveSubsystem) {
    // Note: does not require the drive subsystem itself! It just sets the final point in the drive.
    drive = driveSubsystem;
    tagLimelight = limelight;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (!targetLocked) {
      Optional<Transform2d> robotToScoringLocation = tagLimelight.checkForTag(); //TODO add offset to the getRobotToScoringLocation function
      if (!robotToScoringLocation.isPresent()) {
        robotToScoringLocation = Optional.empty();
        return;
      }
      double latencySeconds = tagLimelight.getLatencySeconds();
      targetLocked = true;
      drive.setLimelightTargetFromTransform(robotToScoringLocation.get(), latencySeconds, true);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return targetLocked;
  }
}

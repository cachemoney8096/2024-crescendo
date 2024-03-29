package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import java.util.Optional;

/** Set the final position in the drive subsystem to the location of the tag the limelight sees */
public class SetTrapLineupPosition extends Command {
  private DriveSubsystem drive;
  private IntakeLimelight tagLimelight;
  private boolean targetLocked = false;

  public SetTrapLineupPosition(IntakeLimelight limelight, DriveSubsystem driveSubsystem) {
    // Note: does not require the drive subsystem itself! It just sets the final point in the drive.
    drive = driveSubsystem;
    tagLimelight = limelight;
  }

  @Override
  public void initialize() {
    targetLocked = false;
  }

  @Override
  public void execute() {
    if (!targetLocked) {
      Optional<Transform2d> robotToScoringLocation = tagLimelight.checkForTag();
      if (!robotToScoringLocation.isPresent()) {
        robotToScoringLocation = Optional.empty();
        return;
      }
      System.out.println("tag present");
      double latencySeconds = tagLimelight.getLatencySeconds();
      targetLocked = true;
      System.out.println("robot to trap: " + robotToScoringLocation.get());
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

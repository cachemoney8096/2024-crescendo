package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.limelightCamMode;
import frc.robot.Constants.limelightLedMode;
import frc.robot.Constants.limelightPipeline;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import java.util.List;
import java.util.Optional;

public class AutoVisionCommand extends SequentialCommandGroup {
  public AutoVisionCommand(
      DriveSubsystem drive,
      Intake intake,
      Conveyor conveyor,
      Elevator elevator,
      Shooter shooter,
      Lights lights,
      IntakeLimelight intakeLimelight) {
    addRequirements(drive, intake, conveyor, elevator);
    addCommands(
        new SequentialCommandGroup(
            new IntakeSequence(intake, elevator, conveyor, shooter, lights),
            new InstantCommand(
                () -> {
                  intakeLimelight.setLimelightValues(
                      limelightLedMode.OFF,
                      limelightCamMode.VISION_PROCESSING,
                      limelightPipeline.NOTE_PIPELINE);
                  Optional<frc.robot.subsystems.intakeLimelight.IntakeLimelight.NoteDetection>
                      latestNoteDetectionOptional = Optional.empty();
                  latestNoteDetectionOptional = intakeLimelight.getNotePos(true);
                  if (latestNoteDetectionOptional.isPresent()) {
                    var latestNoteDetection = latestNoteDetectionOptional.get();
                    final double adjustmentMeters = Units.inchesToMeters(6.0);
                    Translation2d poseAtDetectionToNote =
                        new Translation2d(
                                latestNoteDetection.distanceMeters - adjustmentMeters, 0.0)
                            .rotateBy(Rotation2d.fromDegrees(latestNoteDetection.yawAngleDeg));
                    Pose2d robotPoseAtDetection =
                        drive.getPastBufferedPose(latestNoteDetection.latencySec);
                    Pose2d curPose = drive.getPose();
                    Pose2d goalPose =
                        robotPoseAtDetection.plus(
                            new Transform2d(
                                poseAtDetectionToNote,
                                Rotation2d.fromDegrees(latestNoteDetection.yawAngleDeg)));
                    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                      curPose,
                      goalPose
                    );
                    PathPlannerPath path = new PathPlannerPath(bezierPoints, new PathConstraints(5.0, 6.0, 2*Math.PI, 4*Math.PI), new GoalEndState(2.5, Rotation2d.fromDegrees(latestNoteDetection.yawAngleDeg)));
                    drive.followTrajectoryCommand(path, false);
                  }
                })));
  }
}

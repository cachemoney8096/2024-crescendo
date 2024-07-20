package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SpinMove extends SequentialCommandGroup {
  public enum Direction {
    LEFT,
    RIGHT
  }
  // TODO make this work for red (flip which direction has the radius on the negative side and flip rotational adjustments)
  // TODO end state velocity

  public SpinMove(DriveSubsystem drive, Direction direction, double radiusMeters, double precision) {
    addCommands(
        new InstantCommand(() -> {
          Pose2d curPose = drive.getPose();
          double curX = curPose.getX();
          double h = Math.sqrt(radiusMeters / 2);
          double k = Math.sqrt(radiusMeters / 2);
          double curHeading = drive.getHeadingDegrees();
          List<Pose2d> poses = new LinkedList<Pose2d>();
          h = direction==Direction.LEFT?h*-1:h;
          for (int i = 1; i < precision + 1; i++) {
            double di = (double) i;
            double x = curX - di * (radiusMeters / precision);
            double y = circleY(x, h, k, radiusMeters);
            double headingDegrees = direction==Direction.LEFT?curHeading - di * (180 / precision):curHeading + di * (180 / precision);
            Pose2d pose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(headingDegrees));
            poses.add(pose);
          }
          List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);
          PathPlannerPath path = new PathPlannerPath(bezierPoints, null, new GoalEndState(0.0, Rotation2d.fromDegrees((curHeading+180)%360)));
          path.preventFlipping = true;
          drive.followTrajectoryCommand(path, false);
        }));
  }

  private double circleY(double x, double h, double k, double r) {
    return k - Math.sqrt(Math.pow(r, 2) - Math.pow(x - h, 2));
  }

}

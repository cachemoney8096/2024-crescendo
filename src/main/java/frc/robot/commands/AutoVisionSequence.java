package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;

public class AutoVisionSequence extends SequentialCommandGroup {
  private PathPlannerPath path;
  public AutoVisionSequence(DriveSubsystem drive,
      Intake intake,
      Conveyor conveyor,
      Elevator elevator,
      Shooter shooter,
      Lights lights,
      IntakeLimelight intakeLimelight) {
        addRequirements(drive, intake, conveyor, elevator);
        AutoVisionCommand autoVisionCommand = new AutoVisionCommand(drive, intake, conveyor, elevator, shooter, lights, intakeLimelight);
        addCommands(
          autoVisionCommand, 
          autoVisionCommand.poses.isEmpty()?new InstantCommand():new SequentialCommandGroup(
            new InstantCommand(()->{
              List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                autoVisionCommand.poses.get().getSecond(),
                autoVisionCommand.poses.get().getFirst()
              );
              path = new PathPlannerPath(bezierPoints, new PathConstraints(5.0, 6.0, 2*Math.PI, 4*Math.PI), new GoalEndState(1, autoVisionCommand.poses.get().getFirst().getRotation()));
            }),
            drive.followTrajectoryCommand(path, false)
          )
        );
  }
}

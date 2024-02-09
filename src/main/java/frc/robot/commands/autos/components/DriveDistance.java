package frc.robot.commands.autos.components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer.MatchState;
import frc.robot.subsystems.drive.DriveSubsystem;

/** drives in a direction at a given speed for a certain distance */
public class DriveDistance extends SequentialCommandGroup {
  double distanceTraveledMeters = 0;
  
  Pose2d startPose;
  Translation2d direction;
  Translation2d speeds;
  double distanceToTravel;
  MatchState matchState;

  /**
   * @param normSpeed should be in [0,1]
   * @param xMeters based on 2023 coordinate system (positive X is away from driver station no matter the alliance)
   * @param yMeters based on 2023 coordinate system
   */
  public DriveDistance(
      DriveSubsystem drive,
      double normSpeed,
      double xMeters,
      double yMeters,
      boolean fieldRelative,
      MatchState stateOfMatch) {

    Translation2d desiredTranslation = new Translation2d(xMeters, yMeters);

    this.distanceToTravel = desiredTranslation.getNorm();
    this.direction = desiredTranslation.div(distanceToTravel);
    this.speeds = direction.times(normSpeed);
    this.matchState = stateOfMatch;

    addCommands(
        new InstantCommand(
            () -> {
              this.startPose = drive.getPose();
            }),
        new InstantCommand(
                () -> {
                  final double NOT_ROTATING = 0.0;
                  double xDriveSpeed = matchState.blue ? speeds.getX() : -speeds.getX();
                  double yDriveSpeed = matchState.blue ? speeds.getY() : -speeds.getY();
                  drive.drive(xDriveSpeed, yDriveSpeed, NOT_ROTATING, fieldRelative);
                },
                drive),
        new WaitUntilCommand(
                () -> {
                  Pose2d curPose = drive.getPose();
                  Translation2d movement = curPose.minus(startPose).getTranslation();
                  // This does not ensure that the movement is in the intended direction
                  // but it does ensure that we don't drive further than we intended
                  return movement.getNorm() > distanceToTravel;
                }),
        drive.stopDrivingCommand());
  }
}

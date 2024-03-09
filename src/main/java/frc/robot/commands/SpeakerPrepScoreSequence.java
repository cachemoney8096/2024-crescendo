package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import java.util.Optional;

/**
 * gets the robot ready to shoot a ring into the speaker. gets intake and elevator into position,
 * spins up the shooter
 */
public class SpeakerPrepScoreSequence extends SequentialCommandGroup {

  Optional<Pair<Rotation2d, Double>> tagDetection = Optional.empty();
  double distanceFromSpeakerMeters = 0.0;

  public SpeakerPrepScoreSequence(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      ShooterLimelight limelight,
      DriveSubsystem drive) {

    // Drive is not a requirement!!
    addRequirements(intake, elevator, shooter, conveyor);

    addCommands(
        new GoHomeSequence(intake, elevator, shooter, conveyor, true, false),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new RunCommand(
                () -> {
                  tagDetection = limelight.checkForTag();
                  if (tagDetection.isEmpty()) {
                    return;
                  }

                  System.out.println("tag degrees: " + tagDetection.get().getFirst().getDegrees());
                  drive.setTargetHeadingDegrees(tagDetection.get().getFirst().getDegrees());
                  shooter.setShooterDistance(tagDetection.get().getSecond());
                  shooter.readyToShoot = true;
                })
            .until(
                () -> {
                  ChassisSpeeds currentSpeeds = drive.getCurrentChassisSpeeds();
                  Translation2d currentSpeedsXY =
                      new Translation2d(
                          currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
                  boolean currentlyStatic = currentSpeedsXY.getNorm() < 0.1;
                  return tagDetection.isPresent() && currentlyStatic;
                }));
  }
}

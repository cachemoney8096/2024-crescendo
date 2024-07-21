package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;

public class SpeakerPrepScoreAutoVision extends SequentialCommandGroup{
    Optional<Pair<Rotation2d, Double>> tagDetection = Optional.empty();

    public boolean sawTags = false;

  public SpeakerPrepScoreAutoVision(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      double shooterDistanceMetersFallback,
      ShooterLimelight shooterLimelight,
      DriveSubsystem drive) {

    addRequirements(intake, elevator, shooter, conveyor);

    addCommands(
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SLIGHTLY_UP, true)),
        new RunCommand(
                () -> {
                  ChassisSpeeds currentChassisSpeeds = drive.getCurrentChassisSpeeds();
                  Translation2d driveVelocityMps =
                      new Translation2d(
                          currentChassisSpeeds.vxMetersPerSecond,
                          currentChassisSpeeds.vyMetersPerSecond);

                  Optional<Pair<Rotation2d, Double>> interimTagDetection =
                      shooterLimelight.checkForTag();
                  if (interimTagDetection.isEmpty()) {
                    Pair<Rotation2d, Double> p =
                        ShooterLimelight.getRotationAndDistanceToSpeakerFromPose(
                            drive.getPose(), drive.matchState.isBlue());
                    drive.setTargetHeadingDegrees(
                        p.getFirst().getDegrees()); // TODO uncomment for real match
                    shooter.setShooterDistance(p.getSecond());
                    tagDetection = interimTagDetection;
                    System.out.println("notag");
                  } else {
                    if (driveVelocityMps.getNorm() > 0.02
                        || Math.abs(currentChassisSpeeds.omegaRadiansPerSecond)
                            > Units.degreesToRadians(10)) {
                      // get pose, find limelight stuff from <that> pose use limelight to prep
                      // sequence
                      tagDetection = interimTagDetection;
                      double arbitraryLimelightLatencySec = 0.7;
                      Pose2d futurePose =
                          drive.extrapolatePastPoseBasedOnVelocity(-arbitraryLimelightLatencySec);
                      Pair<Rotation2d, Double> p =
                          ShooterLimelight.getRotationAndDistanceToSpeakerFromPose(
                              futurePose, drive.matchState.isBlue());
                      drive.setTargetHeadingDegrees(p.getFirst().getDegrees());
                      shooter.setShooterDistance(p.getSecond());
                      System.out.println("taginmotion");
                    } else {
                      tagDetection = interimTagDetection;
                      shooterLimelight.resetOdometryDuringPrep(drive);
                      drive.setTargetHeadingDegrees(
                          tagDetection.get().getFirst().getDegrees()
                              + (drive.matchState.isBlue() ? 0.0 : -1.0));
                      shooter.setShooterDistance(
                          tagDetection.get().getSecond()
                              * (drive.matchState.isBlue() ? 0.95 : 1.0));
                      System.out.println("tag");
                    }
                  }
                })
            .until(drive::nearTargetAuto).withTimeout(2.0),
        new WaitUntilCommand(() -> elevator.atDesiredPosition()));
  }
}

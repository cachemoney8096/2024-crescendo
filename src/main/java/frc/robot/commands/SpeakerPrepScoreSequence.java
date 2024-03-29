package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import java.util.Optional;
import java.util.function.BooleanSupplier;

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
      ShooterLimelight shooterLimelight,
      IntakeLimelight intakeLimelight,
      DriveSubsystem drive,
      Lights lights,
      BooleanSupplier driverControllerInput) {

    // Drive is not a requirement!!
    addRequirements(intake, elevator, shooter, conveyor);

    addCommands(
        new InstantCommand(() -> lights.setLEDColor(LightCode.SPEAKER_PREP)),
        new GoHomeSequence(
            intake, elevator, shooter, conveyor, intakeLimelight, true, false, false),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SLIGHTLY_UP, true)),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT)),
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
                    drive.setTargetHeadingDegrees(p.getFirst().getDegrees());
                    shooter.setShooterDistance(p.getSecond());
                    tagDetection = interimTagDetection;
                  } else {
                    if (driveVelocityMps.getNorm() > 0.02
                        || Math.abs(currentChassisSpeeds.omegaRadiansPerSecond)
                            > Units.degreesToRadians(10)) {
                      // get pose, find limelight stuff from <that> pose use limelight to prep
                      // sequence
                      tagDetection = interimTagDetection;
                    double arbitraryLimelightLatencySec = 0.7;
                    Pose2d futurePose = drive.extrapolatePastPoseBasedOnVelocity(-arbitraryLimelightLatencySec);
                    Pair<Rotation2d, Double> p =
                    ShooterLimelight.getRotationAndDistanceToSpeakerFromPose(
                        futurePose, drive.matchState.isBlue());
                      drive.setTargetHeadingDegrees(p.getFirst().getDegrees());
                      shooter.setShooterDistance(p.getSecond());
                    } else {
                      tagDetection = interimTagDetection;
                      shooterLimelight.resetOdometryDuringPrep(drive);
                      drive.setTargetHeadingDegrees(tagDetection.get().getFirst().getDegrees() + (drive.matchState.isBlue() ? 0.0 : -1.0));
                      shooter.setShooterDistance(tagDetection.get().getSecond() * (drive.matchState.isBlue() ? 0.95 : 1.0));
                    }
                  }
                })
            .until(
                () -> {
                  return false;
                }).finallyDo(() -> {shooterLimelight.resetOdometryDuringPrep(drive);System.out.println("rezeroed odemetry in speakerprep");}));
  }
}

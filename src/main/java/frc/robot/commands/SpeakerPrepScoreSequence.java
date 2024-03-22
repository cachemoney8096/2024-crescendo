package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
import frc.robot.utils.DeltaAngleSpeedCalcUtil;

import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * gets the robot ready to shoot a ring into the speaker. gets intake and
 * elevator into position,
 * spins up the shooter
 */
public class SpeakerPrepScoreSequence extends SequentialCommandGroup {

  Optional<Pair<Rotation2d, Double>> tagDetection = Optional.empty();
  double distanceFromSpeakerMeters = 0.0;
  double tempSpeedOfNote = 12.58;

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
              tagDetection = shooterLimelight.checkForTag();
              if (tagDetection.isEmpty()) {
                return;
              }
              ChassisSpeeds currentSpeeds = drive.getCurrentChassisSpeeds();
              DeltaAngleSpeedCalcUtil DeltaAngleSpeedCalcUtil = new DeltaAngleSpeedCalcUtil(tempSpeedOfNote);
              // { delta azimuth DEGREES , delta elevation DEGREES }
              if (tagDetection.isPresent()) {
                Translation2d currentSpeedsXY = new Translation2d(currentSpeeds.vxMetersPerSecond,
                    currentSpeeds.vyMetersPerSecond);
                currentSpeedsXY.rotateBy(tagDetection.get().getFirst());
                Pair<Double, Double> calcDeltaAngles = DeltaAngleSpeedCalcUtil.calcDeltaAngle(currentSpeedsXY.getX(),
                    currentSpeedsXY.getY(), tagDetection.get().getSecond());
                double currentElevationAngle = Math
                    .atan(Constants.SPEAKER_HEIGHT_METERS / tagDetection.get().getSecond());
                new InstantCommand(
                    () -> drive.setTargetHeadingDegrees(drive.getHeadingDegrees() + calcDeltaAngles.getFirst()));
                new InstantCommand(
                    () -> shooter.controlPositionWithAngle((currentElevationAngle + calcDeltaAngles.getSecond()),
                        isScheduled()));
              };
            })
    // .until(
    // () -> {
    // // Translation2d currentSpeedsXY =
    // // new Translation2d(
    // // currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    // // boolean currentlyStatic = currentSpeedsXY.getNorm() < 0.1;
    // })
    );
    // Conveyor.backUpNote(conveyor));
  }
}

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooterLimelight.ShooterLimelightCal;
import frc.robot.utils.MatchStateUtil;

public class LaserFeedPrepScore extends SequentialCommandGroup {
  public LaserFeedPrepScore(
      Elevator elevator,
      Conveyor conveyor,
      Intake intake,
      Shooter shooter,
      DriveSubsystem drive,
      MatchStateUtil matchState,
      IntakeLimelight intakeLimelight,
      Lights lights,
      BooleanSupplier rotationalInputOverride,
      BooleanSupplier cardinalInputOverride) {
    addRequirements(elevator, conveyor, intake, shooter);
    Pose2d cornerPoseBlue = new Pose2d(0.0, 7.0, new Rotation2d(0.0));
    Pose2d cornerPoseRed = new Pose2d(16.0, 7.0, new Rotation2d(0.0));
    Pose2d cornerPose = drive.matchState.isBlue() ? cornerPoseBlue : cornerPoseRed;
    addCommands(
        new InstantCommand(() -> lights.setLEDColor(LightCode.FEED)),
        new GoHomeSequence(
            intake, elevator, shooter, conveyor, intakeLimelight, true, false, false),
        new WaitUntilCommand(elevator::elevatorBelowInterferenceZone),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT_LASER)),
        new RunCommand(() -> {
          if(!rotationalInputOverride.getAsBoolean()){
            Translation2d robotToCorner = cornerPose.getTranslation().minus(drive.getPose().getTranslation());
            double angleToCorner = robotToCorner.getAngle().getDegrees() + 180.0;
            drive.setTargetHeadingDegrees(angleToCorner);
          }
        }).until(()->{return cardinalInputOverride.getAsBoolean() || shooter.shooterMode != ShooterMode.SHOOT_LASER;}).andThen(new InstantCommand(()->System.out.println("override@laser")))
        );
  }

}

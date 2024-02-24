// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import java.util.Optional;

public class RotateToSpeaker extends SequentialCommandGroup {
  Optional<Pair<Rotation2d, Double>> robotToTarget = Optional.empty();

  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker(DriveSubsystem drive, ShooterLimelight limelight) {
    addRequirements(drive);

    addCommands(
        new WaitUntilCommand(
            () -> {
              robotToTarget = limelight.checkForTag();
              return robotToTarget.isPresent();
            }),
        new InstantCommand(
            () -> drive.setTargetHeadingDegrees(robotToTarget.get().getFirst().getDegrees())));
  }
}

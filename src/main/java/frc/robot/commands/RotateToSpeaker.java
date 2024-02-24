// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;
import java.util.Optional;

public class RotateToSpeaker extends SequentialCommandGroup {
  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker(DriveSubsystem drive, ShooterLimelight limelight) {
    addRequirements(drive);

    Optional<Transform2d> robotToTarget = Optional.empty();
    Rotation2d robotToTag;

    addCommands(
        new InstantCommand(
            () -> {
              // robotToTarget = limelight.checkForTag();
            }),
        new ConditionalCommand(null, null, () -> robotToTarget.isPresent()));
  }
}

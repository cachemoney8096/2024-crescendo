// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PIDToPoint extends SequentialCommandGroup {
  PIDController xController =
      new PIDController(
          0.15, Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);
  PIDController yController =
      new PIDController(
          0.15, Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

  Transform2d transform = new Transform2d();

  public PIDToPoint(DriveSubsystem drive) {
    addRequirements(drive);
    this.transform = transform;

    addCommands(
      new InstantCommand(() -> {
        if (drive.targetPose.isEmpty())
        {
          return;
        }
        final Pose2d targetPose2d = drive.targetPose.get();
        drive.setTargetHeadingDegrees(targetPose2d.getRotation().getDegrees());
      }),
      new WaitUntilCommand(() -> {
        if (drive.targetPose.isEmpty())
        {
          return true;
        }
        final Pose2d targetPose2d = drive.targetPose.get();
        final Pose2d currentPose = drive.getPose();
        final double xOutput = xController.calculate(currentPose.getX(), targetPose2d.getX());
        final double yOutput = yController.calculate(currentPose.getY(), targetPose2d.getY());

        drive.rotateOrKeepHeading(xOutput, yOutput, 0.0, true, -1);
        System.out.println("xOutput: " + xOutput);
        System.out.println("yOutput: " + yOutput);
        return (Math.abs(xController.getPositionError()) < 0.0254 && Math.abs(yController.getPositionError()) < 0.0254);
      }),
      new InstantCommand(() -> drive.stopDriving())
    );
  }
}

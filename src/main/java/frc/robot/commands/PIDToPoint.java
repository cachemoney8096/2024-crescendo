// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PIDToPoint extends SequentialCommandGroup {
  PIDController xController = new PIDController(1, 0.0, 0.0); //input meters output -1 to 1
  PIDController yController = new PIDController(1, 0.0, 0.0); //input meters output -1 to 1
  final double feedforwardOutput = 0.2;

  Transform2d transform = new Transform2d();

  static double clamp(double value, double threshold) {
    if (value > threshold) {
      return threshold;
    }
    if (value < -threshold) {
      return -threshold;
    }
    return value;
  }

  public PIDToPoint(DriveSubsystem drive) {
    addRequirements(drive);

    addCommands(
        new InstantCommand(
            () -> {
              System.out.println("target pose: " + drive.targetPose);
              if (drive.targetPose.isEmpty()) {
                return;
              }
              final Pose2d targetPose2d = drive.targetPose.get();
              drive.setTargetHeadingDegrees(targetPose2d.getRotation().getDegrees());
            }),
        new WaitUntilCommand(
            () -> {
              if (drive.targetPose.isEmpty()) {
                System.out.println("drive target pose empty");
                return true;
              }
              final Pose2d targetPose2d = drive.targetPose.get();
              final Pose2d currentPose = drive.getPose();
              double xOutput = xController.calculate(currentPose.getX(), targetPose2d.getX());
              double yOutput = yController.calculate(currentPose.getY(), targetPose2d.getY());
              final double xPosErrorMeters = xController.getPositionError();
              final double yPosErrorMeters = yController.getPositionError();
              final Translation2d errorMeters = new Translation2d(xPosErrorMeters, yPosErrorMeters);
              final Translation2d errorDirection = errorMeters.div(errorMeters.getNorm());
              final double xFfOutput = errorDirection.getX() * feedforwardOutput;
              final double yFfOutput = errorDirection.getY() * feedforwardOutput;
              xOutput += xFfOutput;
              yOutput += yFfOutput;

              double xOutputClamped = clamp(xOutput, 0.4);
              double yOutputClamped = clamp(yOutput, 0.4);

              drive.rotateOrKeepHeading(xOutputClamped, yOutputClamped, 0.0, true, -1);

              SmartDashboard.putNumber("x error", xController.getPositionError());
              SmartDashboard.putNumber("y error", yController.getPositionError());
              return (Math.abs(xController.getPositionError()) < 0.01
                  && Math.abs(yController.getPositionError()) < 0.01);
            }),
        new InstantCommand(() -> drive.stopDriving()));
  }
}

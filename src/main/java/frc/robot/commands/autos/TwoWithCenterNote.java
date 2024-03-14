// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.SpeakerPrepScoreSequence;
import frc.robot.commands.SpeakerShootSequence;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;

public class TwoWithCenterNote extends SequentialCommandGroup {

  public PathPlannerPath middleNoteAuto;

  /** Creates a new TwoWithCenterNote. */
  public TwoWithCenterNote(
      DriveSubsystem drive,
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      ShooterLimelight limelight,
      Lights lights) {
    NamedCommands.registerCommand(
        "goHomeWithShooterSpunUp",
        new GoHomeSequence(intake, elevator, shooter, conveyor, true, false, false));
    NamedCommands.registerCommand(
        "intakeNote",
        new IntakeSequence(intake, elevator, conveyor, shooter, lights).withTimeout(5.0));
    middleNoteAuto = PathPlannerPath.fromPathFile("Middle Note Auto");
    addCommands(
        drive.followTrajectoryCommand(middleNoteAuto, true),
        new SpeakerPrepScoreSequence(
                intake,
                elevator,
                shooter,
                conveyor,
                limelight,
                drive,
                () -> {
                  return false;
                })
            .withTimeout(5.0),
        new WaitUntilCommand(() -> shooter.isShooterSpunUp()),
        new SpeakerShootSequence(conveyor, shooter, elevator, drive));
  }
}

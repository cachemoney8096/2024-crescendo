package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
      Lights lights) {
    addRequirements(elevator, conveyor, intake, shooter);
    addCommands(
        new InstantCommand(() -> lights.setLEDColor(LightCode.FEED)),
        new GoHomeSequence(
            intake, elevator, shooter, conveyor, intakeLimelight, true, false, false),
        new WaitUntilCommand(elevator::elevatorBelowInterferenceZone),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT_LASER)));
  }
}

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.MatchState;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

public class FeedPrepScore extends SequentialCommandGroup {
  public FeedPrepScore(Elevator elevator, Conveyor conveyor, Intake intake, Shooter shooter, DriveSubsystem drive,
      MatchState matchState) {
    addRequirements(elevator, conveyor, intake, shooter);
    addCommands(
        new GoHomeSequence(intake, elevator, shooter, conveyor, true),
        new WaitUntilCommand(elevator::elevatorBelowInterferenceZone),
        new ConditionalCommand(
            new InstantCommand(() -> drive.setTargetHeadingDegrees(315.0)),
            new InstantCommand(() -> drive.setTargetHeadingDegrees(225.0)),
            () -> matchState.blue),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.SHOOT_CLEAR_STAGE)),
        new InstantCommand(() -> conveyor.startBackRollers(1.0))
        );
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Prepares to score in the amp. This command group safely moves the conveyor and elevator to the
 * score amp position.
 */
public class AmpPrepScore extends SequentialCommandGroup {
  /** Creates a new AmpPreScore. */
  public AmpPrepScore(
      Elevator elevator, Conveyor conveyor, Intake intake, Shooter shooter, DriveSubsystem drive, Lights lights) {
    addRequirements(conveyor, elevator, intake, shooter);

    SequentialCommandGroup moveWhenNotSafe =
        new SequentialCommandGroup(
            new InstantCommand(() -> drive.setTargetHeadingDegrees(90)),
            new InstantCommand(() -> shooter.setShooterMode(Shooter.ShooterMode.IDLE)),
            new SafeDeploy(intake, elevator, true),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            new WaitUntilCommand(shooter::clearOfConveyorZone),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP, true)),
            new WaitUntilCommand(elevator::elevatorAboveIntakeInterferenceZone));

    addCommands(
        new InstantCommand(() -> lights.setLEDColor(LightCode.AMP_PREP)),
        new InstantCommand(() -> conveyor.stopRollers()),
        new ConditionalCommand(
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SCORE_AMP, true)),
            moveWhenNotSafe,
            () -> elevator.elevatorAboveIntakeInterferenceZone()),
        new InstantCommand(
            () -> intake.setDesiredIntakePosition(IntakePosition.ALMOST_CLEAR_OF_CONVEYOR)));
  }
}

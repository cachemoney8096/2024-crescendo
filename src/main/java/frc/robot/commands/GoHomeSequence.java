package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.limelightCamMode;
import frc.robot.Constants.limelightLedMode;
import frc.robot.Constants.limelightPipeline;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * Brings the elevator, intake, and shooter to their respective home positions in a "smart" way so
 * they do not end up attacking each other accidentally. Ends before everything is fully home.
 */
public class GoHomeSequence extends SequentialCommandGroup {
  public GoHomeSequence(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      IntakeLimelight intakeLimelight,
      boolean spinUpShooter,
      boolean stowIntake,
      boolean homeElevator) {
    final ShooterMode desiredShooterMode = spinUpShooter ? ShooterMode.SPIN_UP : ShooterMode.IDLE;
    final ElevatorPosition desiredElevatorPosition =
        homeElevator ? ElevatorPosition.HOME : ElevatorPosition.SLIGHTLY_UP;
    final SequentialCommandGroup goHomeWhenSafe =
        new SequentialCommandGroup(
            new ConditionalCommand(
                new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
                new InstantCommand(),
                () -> stowIntake),
            new InstantCommand(() -> shooter.setShooterMode(desiredShooterMode)),
            new InstantCommand(() -> elevator.setDesiredPosition(desiredElevatorPosition, true)));

    final SequentialCommandGroup goHomeWhenNotSafe =
        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setShooterMode(desiredShooterMode)),
            new WaitUntilCommand(shooter::clearOfConveyorZone),
            new SafeDeploy(intake, elevator, stowIntake),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)),
            new WaitUntilCommand(elevator::elevatorBelowInterferenceZone),
            new ConditionalCommand(
                new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)),
                new InstantCommand(),
                () -> stowIntake));

    addRequirements(intake, elevator, shooter, conveyor);
    addCommands(
        new InstantCommand(conveyor::stopRollers),
        new InstantCommand(intake::stopRollers),
        new InstantCommand(
            () ->
                intakeLimelight.setLimelightValues(
                    limelightLedMode.OFF,
                    limelightCamMode.DRIVER_CAMERA,
                    limelightPipeline.TAG_PIPELINE)),
        new ConditionalCommand(
            goHomeWhenSafe, goHomeWhenNotSafe, () -> elevator.elevatorBelowInterferenceZone()));
  }
}

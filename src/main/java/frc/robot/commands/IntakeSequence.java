package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * Puts the intake into the deployed position then gets elevator to home position Checks that
 * everything is in position to intake then starts rollers Intakes until the conveyor recieves the
 * game piece Stops and stowes intake
 */
public class IntakeSequence extends SequentialCommandGroup {
  public IntakeSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    addRequirements(intake, elevator, conveyor, shooter);

    addCommands(
        new InstantCommand(() -> SmartDashboard.putBoolean("Have Note", false)),
        new InstantCommand(
            () -> {
              shooter.setShooterMode(ShooterMode.IDLE);
            }),
        new WaitUntilCommand(shooter::clearOfConveyorZone),
        new SafeDeploy(intake, elevator),
        new ConditionalCommand(
            new InstantCommand(),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            elevator::elevatorBelowInterferenceZone),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new WaitUntilCommand(intake::nearDeployed),
        new InstantCommand(intake::startRollers),
        Conveyor.receive(conveyor),
        new InstantCommand(
            () -> {
              intake.stopRollers();
            },
            intake));
        // new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.STOWED)));
  }
}

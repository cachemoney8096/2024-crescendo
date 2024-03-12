package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

/**
 * Puts the intake into the deployed position then gets elevator to home
 * position Checks that
 * everything is in position to intake then starts rollers Intakes until the
 * conveyor recieves the
 * game piece Stops and stowes intake
 */
public class AutoIntakeSequence extends SequentialCommandGroup {
  public AutoIntakeSequence(Intake intake, Elevator elevator, Conveyor conveyor, Lights lights) {
    addRequirements(intake, elevator, conveyor, lights);

    addCommands(
        new InstantCommand(() -> lights.toggleCode(LightCode.INTAKING)),
        new InstantCommand(() -> SmartDashboard.putBoolean("Have Note", false)),
        new InstantCommand(() -> intake.setDesiredIntakePosition(IntakePosition.DEPLOYED)),
        new WaitUntilCommand(intake::nearDeployed),
        new InstantCommand(intake::startRollers),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.INTAKING, true)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        Conveyor.startReceive(conveyor),
        new ParallelCommandGroup(
            Conveyor.rumbleBriefly(conveyor),
            new InstantCommand(intake::stopRollers, intake),
            Conveyor.finishReceive(conveyor)),
        new InstantCommand(() -> lights.toggleCode(LightCode.INTAKING)));
  }
}

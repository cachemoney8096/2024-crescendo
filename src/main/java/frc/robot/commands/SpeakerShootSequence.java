package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * shoots a ring into the speaker once the shooter is in position and spun up, then brings the robot
 * back to rest. this assumes that the robot is already prepared to shoot
 */
public class SpeakerShootSequence extends SequentialCommandGroup {
  public SpeakerShootSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    addRequirements(conveyor, shooter);
    addCommands(
        new WaitUntilCommand(
            () -> {
              return shooter.atDesiredPosition() && shooter.isShooterSpunUp();
            }),
        Conveyor.shoot(conveyor),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.IDLE)));
  }

  /** runs SpeakerShootSequence but adds the layer that if the robot is interrupted, then everything is brought to the prep state */
  public static Command interruptibleSpeakerShootSequence(Intake intake, Elevator elevator, Conveyor conveyor, Shooter shooter) {
    return new SpeakerShootSequence(intake, elevator, conveyor, shooter)
        .finallyDo(
            (boolean interrupted) -> {
              SpeakerPrepScoreSequence.interruptibleSpeakerPrepScoreSequence(intake, elevator, conveyor, shooter);
            });
  }
}

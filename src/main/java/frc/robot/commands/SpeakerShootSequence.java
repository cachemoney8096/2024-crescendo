package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.conveyer.Conveyer;

public class SpeakerShootSequence extends SequentialCommandGroup {
  public SpeakerShootSequence(Conveyer conveyor, Shooter shooter) {
    addRequirements(conveyor, shooter);
    addCommands(
        Conveyer.shoot(conveyor),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.IDLE)));
  }
}

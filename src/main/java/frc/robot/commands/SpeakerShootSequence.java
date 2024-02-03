package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.conveyor.Conveyor;

public class SpeakerShootSequence extends SequentialCommandGroup {
  public SpeakerShootSequence(Conveyor conveyor, Shooter shooter) {
    addRequirements(conveyor, shooter);
    addCommands(
        new WaitUntilCommand(() -> {
          return shooter.atDesiredPosition() && shooter.isShooterSpunUp();
        }),
        Conveyor.shoot(conveyor),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.IDLE)));
  }
}

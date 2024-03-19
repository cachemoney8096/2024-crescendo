package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;
import frc.robot.subsystems.shooter.ShooterCal;

/**
 * shoots a ring into the speaker once the shooter is in position and spun up, then brings the robot
 * back to rest. this assumes that the robot is already prepared to shoot
 */
public class SpeakerShootSequence extends SequentialCommandGroup {
  public SpeakerShootSequence(
      Conveyor conveyor,
      Shooter shooter,
      Elevator elevator,
      DriveSubsystem drive,
      boolean waitUntilRotated) {
    addRequirements(conveyor, shooter);
    addCommands(
        new WaitUntilCommand(
            () -> {
              return elevator.atDesiredPosition() &&
                     shooter.atDesiredPosition() &&
                     shooter.isShooterSpunUp() &&
                     (!waitUntilRotated ||
                      drive.getDiffCurrentTargetYawDeg() < ShooterCal.ROBOT_HEADING_MARGIN_TO_SHOOT_DEGREES);
            }).withTimeout(2.0),
        Conveyor.shoot(conveyor),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.IDLE)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)));
  }

  /** Defaults to <b> not </b> waiting until rotated */
  public SpeakerShootSequence(
      Conveyor conveyor, Shooter shooter, Elevator elevator, DriveSubsystem drive) {
    this(conveyor, shooter, elevator, drive, false);
  }
}

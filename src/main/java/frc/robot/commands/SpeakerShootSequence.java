package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
      BooleanSupplier waitUntilRotatedFunc) {
    addRequirements(conveyor, shooter);
    addCommands(
        new InstantCommand(
            () -> {
              System.out.println("is teleop in shoot" + waitUntilRotatedFunc.getAsBoolean());
            }),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.SPEAKER_PREP, true)),
        new ConditionalCommand(
            new PrintCommand("Waiting for robot heading")
                .andThen(
                    new WaitUntilCommand(
                        () ->
                            drive.getDiffCurrentTargetYawDeg()
                                < ShooterCal.ROBOT_HEADING_MARGIN_TO_SHOOT_DEGREES))
                .withTimeout(2.0)
                .andThen(new PrintCommand("Done waiting for target heading")),
            new InstantCommand(),
            waitUntilRotatedFunc),
        new PrintCommand("Waiting for shooter at desired and spun up"),
        new WaitUntilCommand(
            () -> {
              return shooter.atDesiredPosition() && shooter.isShooterSpunUp();
            }),
        new PrintCommand("Done waiting for shooter at desired and spun up"),
        new PrintCommand("Waiting for elevator at desired"),
        new WaitUntilCommand(() -> elevator.atDesiredPosition()),
        new PrintCommand("Done waiting for elevator at desired"),
        Conveyor.shoot(conveyor),
        new InstantCommand(() -> shooter.setShooterMode(ShooterMode.IDLE)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorPosition.HOME, true)));
  }

  /** Defaults to <b> not </b> waiting until rotated */
  public SpeakerShootSequence(
      Conveyor conveyor, Shooter shooter, Elevator elevator, DriveSubsystem drive) {
    this(conveyor, shooter, elevator, drive, () -> false);
  }
}

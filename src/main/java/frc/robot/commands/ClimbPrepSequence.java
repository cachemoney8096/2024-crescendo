package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.limelightCamMode;
import frc.robot.Constants.limelightLedMode;
import frc.robot.Constants.limelightPipeline;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeLimelight.IntakeLimelight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterMode;

/**
 * Gets the robot ready to climb by moving the intake, elevator, and shooter to their appropriate
 * positions. Does not wait until everything is at their desired positions!
 */
public class ClimbPrepSequence extends SequentialCommandGroup {
  public ClimbPrepSequence(
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      IntakeLimelight intakeLimelight,
      Lights lights) {
    final SequentialCommandGroup restOfPrep =
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    intakeLimelight.setLimelightValues(
                        limelightLedMode.OFF,
                        limelightCamMode.VISION_PROCESSING,
                        limelightPipeline.TAG_PIPELINE)),
            new InstantCommand(() -> shooter.setShooterMode(ShooterMode.PRELATCH)),
            new WaitUntilCommand(shooter::clearOfConveyorZone),
            new SafeDeploy(intake, elevator, false),
            new WaitUntilCommand(intake::clearOfConveyorZone),
            new InstantCommand(
                () -> elevator.setDesiredPosition(ElevatorPosition.PRE_CLIMB, true)));
    addRequirements(intake, elevator, shooter, conveyor);

    addCommands(
        new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMB_PREP)),
        new ParallelCommandGroup(Conveyor.crushNote(conveyor), restOfPrep));
  }
}

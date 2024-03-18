package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterLimelight.ShooterLimelight;

/** Auto mode for notes 8-6-7 */
public class SideSixToEight extends SequentialCommandGroup {
  /** Right on blue, left on red */
  public SideSixToEight(
      DriveSubsystem drive,
      Intake intake,
      Elevator elevator,
      Shooter shooter,
      Conveyor conveyor,
      ShooterLimelight limelight,
      boolean isRed) {

    addRequirements(drive, intake, elevator, shooter, conveyor, limelight);

    /** Initialize sequential commands that run for the "15 second autonomous phase" */
    addCommands(
        new ConditionalCommand(
            new PathPlannerAuto("LEFT 8-6-7 RED"),
            new PathPlannerAuto("RIGHT 8-6-7"),
            () -> isRed));
  }
}

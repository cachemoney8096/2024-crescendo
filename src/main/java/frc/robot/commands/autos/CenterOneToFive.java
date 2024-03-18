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

/** Auto mode for notes 2-1-3-5-4 */
public class CenterOneToFive extends SequentialCommandGroup {
  public CenterOneToFive(
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
            new PathPlannerAuto("CENTER 1-2-3-4-5 RED"),
            new PathPlannerAuto("CENTER 1-2-3-4-5"),
            () -> isRed));
  }
}

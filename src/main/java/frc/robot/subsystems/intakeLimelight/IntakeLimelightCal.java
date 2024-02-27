package frc.robot.subsystems.intakeLimelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeLimelightCal {
  public static final double LIMELIGHT_YAW_DEGREES = 0.0;
  /** offset from trap apriltag */
  public static final Transform2d TRAP_OFFSET = 
    new Transform2d(new Translation2d(-Units.inchesToMeters(11.5), 0), new Rotation2d(0));

  public static final double LIMELIGHT_DETECTION_OFFSET_DEGREES = 0.0;
}

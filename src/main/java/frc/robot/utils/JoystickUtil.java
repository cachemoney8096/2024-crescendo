package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class JoystickUtil {
  /** Radius on controller less than 0.1 results in output of zero */
  public static final double DEADBAND = 0.1;

  /** Return X and Y outputs for drive */
  public static Pair<Double, Double> computeDriveXY(
      CommandXboxController controller, boolean fieldRelative, boolean isBlue) {
    final double xIn = isBlue || !fieldRelative ? -controller.getLeftY() : controller.getLeftY();
    final double yIn = isBlue || !fieldRelative ? -controller.getLeftX() : controller.getLeftX();
    return computeDriveXY(xIn, yIn);
  }

  public static Pair<Double, Double> computeDriveXY(double xIn, double yIn) {
    double magnitude =
        Math.hypot(
            xIn, yIn); // if corner is 1.0,1.0, then change this line to Math.max(Math.abs(xIn),
    // Math.abs(yIn))
    if (magnitude < DEADBAND) {
      return Pair.of(0.0, 0.0);
    }
    magnitude = MathUtil.applyDeadband(magnitude, DEADBAND, 1.0);
    magnitude = magnitude * magnitude;
    final var direction = new Rotation2d(xIn, yIn);
    final var outputXY = new Translation2d(magnitude, 0.0).rotateBy(direction);
    return Pair.of(outputXY.getX(), outputXY.getY());
  }

  public static double squareAxis(double input) {
    return (input * input) * Math.signum(input);
  }
}

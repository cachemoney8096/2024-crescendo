import frc.robot.utils.JoystickUtil;

import org.junit.*;
import org.junit.jupiter.api.Test;

public class JoystickTesting {

  static final double EPSILON = 1e-4;

  void testDeadband(double xIn, double yIn, double xExpected, double yExpected) {
    var deadbandResult = JoystickUtil.computeDriveXY(xIn, yIn);
    Assert.assertEquals(deadbandResult.getFirst(), xExpected, EPSILON);
    Assert.assertEquals(deadbandResult.getSecond(), yExpected, EPSILON);
  }

  @Test
  void allTests11() {
    // Simple tests
    testDeadband(0, 0, 0, 0);
    testDeadband(1, 0, 1, 0);
    testDeadband(-1, 0, -1, 0);
    testDeadband(0, 1, 0, 1);

    // Within deadband
    testDeadband(0.05, 0.05, 0, 0);
    testDeadband(0.05, 0, 0, 0);
    testDeadband(0.0, 0.05, 0, 0);
    testDeadband(-0.05, -0.05, 0, 0);

    // Corner tests
    testDeadband(0.7071, 0.7071, 0.7071, 0.7071);
    testDeadband(-0.7071, -0.7071, -0.7071, -0.7071);
    testDeadband(-0.7071, 0.7071, -0.7071, 0.7071);
    testDeadband(0.7071, -0.7071, 0.7071, -0.7071);

    // Basic tests not at limits
    testDeadband(0.55, 0.0, 0.25, 0.0); // 0.55 is halfway past deadband, with squaring comes out to 0.25

    // Test at 45s
    testDeadband(0.7071 * 0.55, -0.7071 * 0.55, 0.7071 * 0.25, -0.7071 * 0.25);

    // Test at other angle
    // magnitude 0.447214
    // after deadband ramping, magnitude 0.38579
    // after squaring, magnitude 0.14883614925
    // atan(2) = 63.43494 deg
    // sin(63.43494 deg) = 0.894427
    // cos(63.43494 deg) = 0.44721
    testDeadband(-0.4, 0.2, -0.14883614925 * 0.894427, 0.14883614925 * 0.44721);
  }
}

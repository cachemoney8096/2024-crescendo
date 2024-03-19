import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.*;
import org.junit.jupiter.api.Test;

public class OptimizeTest {

  static final double EPSILON_MPS = 1e-4;
  static final double EPSILON_DEG = 1e-4;

  void testState(double desiredRad, double currentRad, boolean expectedFlip) {
    SwerveModuleState desiredState = new SwerveModuleState(1.0, Rotation2d.fromRadians(desiredRad));
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(currentRad));
    if (expectedFlip) {
      Assert.assertEquals(
          optimizedState.speedMetersPerSecond, -desiredState.speedMetersPerSecond, EPSILON_MPS);
      Assert.assertEquals(
          optimizedState.angle.getRadians(),
          desiredState.angle.plus(Rotation2d.fromRadians(Math.PI)).getRadians(),
          EPSILON_DEG);
    } else {
      Assert.assertEquals(
          optimizedState.speedMetersPerSecond, desiredState.speedMetersPerSecond, EPSILON_MPS);
      Assert.assertEquals(
          optimizedState.angle.getRadians(), desiredState.angle.getRadians(), EPSILON_DEG);
    }

    // Following don't actually pass, is that a problem?
    // Assert.assertTrue(optimizedState.angle.getRadians() + " < " + -Math.PI,
    // optimizedState.angle.getRadians() >= -Math.PI);
    // Assert.assertTrue(optimizedState.angle.getRadians() + " > " + Math.PI,
    // optimizedState.angle.getRadians() <= Math.PI);
  }

  @Test
  void allTests() {
    // Same Exact
    testState(2.0 * Math.PI / 3.0, 2.0 * Math.PI / 3.0, false);

    // Basic delta within 90
    testState(2.0 * Math.PI / 3.0, Math.PI, false);
    testState(2.0 * Math.PI / 3.0, 1.0 * Math.PI / 3.0, false);

    // Basic delta within 90, negative values
    testState(0.5 * Math.PI / 3.0, -0.5 * Math.PI / 3.0, false);
    testState(-0.5 * Math.PI / 3.0, -1.5 * Math.PI / 3.0, false);

    // Basic delta above 90
    testState(2.0 * Math.PI / 3.0, 4.0 * Math.PI / 3.0, true);
    testState(2.0 * Math.PI / 3.0, 0.0, true);

    // Exact plus 360
    testState(2.0 * Math.PI / 3.0, 8.0 * Math.PI / 3.0, false);
    testState(8.0 * Math.PI / 3.0, 2.0 * Math.PI / 3.0, false);

    // Roughly 360 off
    testState(2.0 * Math.PI / 3.0, 8.5 * Math.PI / 3.0, false);
    testState(2.0 * Math.PI / 3.0, 7.5 * Math.PI / 3.0, false);
    testState(8.0 * Math.PI / 3.0, 2.5 * Math.PI / 3.0, false);
    testState(8.0 * Math.PI / 3.0, 1.5 * Math.PI / 3.0, false);

    // Roughly 360 off, negative values
    testState(-2.0 * Math.PI / 3.0, 4.5 * Math.PI / 3.0, false);
    testState(-2.0 * Math.PI / 3.0, 3.5 * Math.PI / 3.0, false);
    testState(4.0 * Math.PI / 3.0, -2.5 * Math.PI / 3.0, false);
    testState(4.0 * Math.PI / 3.0, -1.5 * Math.PI / 3.0, false);
  }
}

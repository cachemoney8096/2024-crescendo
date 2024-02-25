import edu.wpi.first.math.Pair;
import frc.robot.utils.DeltaAngleSpeedCalcUtil;
import org.junit.jupiter.api.Test;

public class DeltaAngleSpeedCalcTest {
  /**
   * all angles output as such:
   * { delta azimuth DEGREES , delta elevation DEGREES }
   */
  private static double arbitraryTangentialVelocityMPS = 3;
  private static double arbitraryRadialVelocityMPS = 3;
  private static double distFromSpeakerMeters = 1.5;

  @Test
  void test() {
    assert true;
  }

  @Test
  public void randomVelocitySigns() {
    DeltaAngleSpeedCalcUtil deltaAngleSpeedCalc = new DeltaAngleSpeedCalcUtil(9.144);
    double arbitraryTangentialVelocityMPS = 3;
    double arbitraryRadialVelocityMPS = 3;
    double distFromSpeakerMeters = 1.5;
    // moving right and backward with radial and tangential velocities respectively

    Pair<Double, Double> calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS,
        arbitraryRadialVelocityMPS, distFromSpeakerMeters);

    // angles gathered by hand: {18.16383517, 18.06236439}
    Pair<Double, Double> calcAnglesByHand = new Pair<Double, Double>(18.16383517, 18.06236439);
    assert Math.abs(calcAnglesByHand.getFirst() - calculatedAngles.getFirst()) <= 0.0001;
    assert Math.abs(calcAnglesByHand.getSecond() - calculatedAngles.getSecond()) <= 0.0001;

    // Test with negative velocities (moving left and forwards?)
    arbitraryRadialVelocityMPS = -3;
    arbitraryTangentialVelocityMPS = -3;
    distFromSpeakerMeters = 1.5;
    calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS, arbitraryRadialVelocityMPS,
        distFromSpeakerMeters);

    // angles gathered by hand { -18.1638517, -12.31285929}
    calcAnglesByHand = new Pair<Double, Double>(-18.1638517, -12.31285929);

    assert Math.abs(calcAnglesByHand.getFirst() - calculatedAngles.getFirst()) <= 0.0001;
    assert Math.abs(calcAnglesByHand.getSecond() - calculatedAngles.getSecond()) <= 0.0001;

  }

  // Test with different shooter speeds?
  @Test
  public void randomNoteSpeeds() {
    // note speed = 12m/sec
    DeltaAngleSpeedCalcUtil deltaAngleSpeedCalc = new DeltaAngleSpeedCalcUtil(12);
    double arbitraryTangentialVelocityMPS = 3;
    double arbitraryRadialVelocityMPS = 3;
    double distFromSpeakerMeters = 1.5;

    Pair<Double, Double> calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS,
        arbitraryRadialVelocityMPS, distFromSpeakerMeters);

    // angles gathered by hand: {14.03624347, 13.20854334 }
    Pair<Double, Double> calcAnglesByHand = new Pair<Double, Double>(14.03624347, 13.20854334);

    assert Math.abs(calcAnglesByHand.getFirst() - calculatedAngles.getFirst()) <= 0.00001;
    assert Math.abs(calcAnglesByHand.getSecond() - calculatedAngles.getSecond()) <= 0.00001;

    // note speed = 15m/sec
    deltaAngleSpeedCalc = new DeltaAngleSpeedCalcUtil(15);
    calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS,
        arbitraryRadialVelocityMPS, distFromSpeakerMeters);
    // angles gathered by hand: {11.30993247, 10.27700868 }
    calcAnglesByHand = new Pair<Double, Double>(11.30993247, 10.27700868);

    assert Math.abs(calcAnglesByHand.getFirst() - calculatedAngles.getFirst()) <= 0.00001;
    assert Math.abs(calcAnglesByHand.getSecond() - calculatedAngles.getSecond()) <= 0.00001;

  }

  // test w/ different distances from speaker
  // (only elevation will have a change. azimuth does not get affected by this
  // whatsoever.)
  @Test
  public void randomDistancesFromSpeaker() {
    DeltaAngleSpeedCalcUtil deltaAngleSpeedCalc = new DeltaAngleSpeedCalcUtil(9.144);
    // distance from speaker = 5meters
    distFromSpeakerMeters = 5;
    Pair<Double, Double> calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS,
        arbitraryRadialVelocityMPS, distFromSpeakerMeters);

    // angles gathered by hand: {18.16383517, 9.86034327 }
    Pair<Double, Double> calcAnglesByHand = new Pair<Double, Double>(18.16383517, 9.86034327);

    assert Math.abs(calcAnglesByHand.getFirst() - calculatedAngles.getFirst()) <= 0.00001;
    assert Math.abs(calcAnglesByHand.getSecond() - calculatedAngles.getSecond()) <= 0.00001;

    // distance from speaker = 5meters
    distFromSpeakerMeters = 7;
    calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS,
        arbitraryRadialVelocityMPS, distFromSpeakerMeters);

    // angles gathered by hand: {18.16383517, 7.43475071 }
    calcAnglesByHand = new Pair<Double, Double>(18.16383517, 7.43475071);

    assert Math.abs(calcAnglesByHand.getFirst() - calculatedAngles.getFirst()) <= 0.00001;
    assert Math.abs(calcAnglesByHand.getSecond() - calculatedAngles.getSecond()) <= 0.00001;
  }

}

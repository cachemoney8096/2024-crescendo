import edu.wpi.first.math.Pair;
import frc.robot.utils.DeltaAngleSpeedCalcUtil;
import org.junit.jupiter.api.Test;

public class DeltaAngleSpeedCalcTest {
  @Test
  void test() {
    DeltaAngleSpeedCalcUtil deltaAngleSpeedCalc = new DeltaAngleSpeedCalcUtil();
    double arbitraryTangentialVelocityMPS = 3;
    double arbitraryRadialVelocityMPS = 3;
    double distFromSpeakerMeters = 1.5;

    // { delta azimuth, delta elevation }
    Pair<Double, Double> calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS,
        arbitraryRadialVelocityMPS, distFromSpeakerMeters);
    System.out.println(calculatedAngles.getFirst() + ", " + calculatedAngles.getSecond());

    // angles gathered by hand: {18.16383517, -18.06236439}
    Pair<Double, Double> calcAnglesByHand = new Pair<Double, Double>(18.16383517, 18.06236439);

    assert Math.abs(calcAnglesByHand.getFirst() - calculatedAngles.getFirst()) <= 0.00001;
    assert Math.abs(calcAnglesByHand.getSecond() - calculatedAngles.getSecond()) <= 0.00001;

    // Test with negative velocities (moving left?)
    arbitraryRadialVelocityMPS = -3;
    arbitraryTangentialVelocityMPS = -3;
    distFromSpeakerMeters = 1.5;
    calculatedAngles = deltaAngleSpeedCalc.calcDeltaAngle(arbitraryTangentialVelocityMPS, arbitraryRadialVelocityMPS,
        distFromSpeakerMeters);
    System.out.println(calculatedAngles.getFirst() + ", " + calculatedAngles.getSecond());
  }
}

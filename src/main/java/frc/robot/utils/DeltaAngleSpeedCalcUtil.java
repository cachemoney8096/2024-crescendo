package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;

public class DeltaAngleSpeedCalcUtil {
  // value pulled from game manual
  private static final double SPEAKER_HEIGHT_METERS = Units.inchesToMeters(77.9528);
  // guess-timated value for now
  public static double SHOOTER_SPEED_MPS;
  public DeltaAngleSpeedCalcUtil(double speed) {
    SHOOTER_SPEED_MPS = speed;
  };
  /**
   *
   * @param tangentialVelocityMPS: see google doc
   * @param radialVelocityMPS:     see google doc
   * @param distFromSpeakerMeters: distance from speaker :)
   * @return: delta of azimuth and elevation angle degrees (change in angle)
   */
  public Pair<Double, Double> calcDeltaAngle(
      double tangentialVelocityMPS,
      double radialVelocityMPS,
      double distFromSpeakerMeters) {

    /**
     * :)))))))))))))))))))
     * https://docs.google.com/document/d/1rIMmzDA1EMH7NQexIMrNYAD-_KFLe9cL4zPLXdl0lek/edit?usp=sharing
     */

    double deltaAzimuthAngleDegrees = Math.toDegrees(Math.atan(tangentialVelocityMPS / SHOOTER_SPEED_MPS));

    double currentElevationAngleDegrees = Math.atan(SPEAKER_HEIGHT_METERS / distFromSpeakerMeters);
  
    double verticalElevationVelocityMPS = Math.tan(currentElevationAngleDegrees)
        * (SHOOTER_SPEED_MPS * Math.cos(currentElevationAngleDegrees));

    double calculatedElevationFraction = (verticalElevationVelocityMPS)/ ((SHOOTER_SPEED_MPS * Math.cos(currentElevationAngleDegrees)) - radialVelocityMPS);
    double calculatedElevationAngleDegrees = Math.toDegrees(Math.atan(calculatedElevationFraction));

    double deltaElevationAngleDegrees = calculatedElevationAngleDegrees - Math.toDegrees(currentElevationAngleDegrees);

    Pair<Double, Double> calculatedDeltaAngleGivenSpeeds = new Pair<Double, Double>(deltaAzimuthAngleDegrees,
        (-1 * deltaElevationAngleDegrees));

    return calculatedDeltaAngleGivenSpeeds;

  }
}

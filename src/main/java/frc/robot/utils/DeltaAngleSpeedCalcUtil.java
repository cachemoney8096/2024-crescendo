package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DeltaAngleSpeedCalcUtil {
  private static final double SPEAKER_HEIGHT_INCHES = Units.inchesToMeters(77.9528);
  private static final double SHOOTER_SPEED_MPS = Constants.PLACEHOLDER_DOUBLE;

  /**
   *
   * @param tangentialVelocityMPS: see google doc
   * @param radialVelocityMPS: see google doc
   * @param distFromSpeakerMeters: distance from speaker :)
   * @param currentAzimuthAngleDegrees: see google doc
   * @return: delta of azimuth and elevation angle degrees (change in angle)
   */
  public static Pair<Double, Double> calcDeltaAngle(
      double tangentialVelocityMPS,
      double radialVelocityMPS,
      double distFromSpeakerMeters,
      double currentAzimuthAngleDegrees) {


    /** 
     * :)))))))))))))))))))
     * https://docs.google.com/document/d/1rIMmzDA1EMH7NQexIMrNYAD-_KFLe9cL4zPLXdl0lek/edit?usp=sharing
     */
    double calculatedAzimuthAngleDegrees = Math.atan(tangentialVelocityMPS / SHOOTER_SPEED_MPS);
    double deltaAzimuthAngleDegrees = currentAzimuthAngleDegrees - calculatedAzimuthAngleDegrees;

    double currentElevationAngleDegrees = Math.atan(SPEAKER_HEIGHT_INCHES / distFromSpeakerMeters);
    double verticalElevationVelocityMPS = Math.tan(currentElevationAngleDegrees) * (SHOOTER_SPEED_MPS * Math.cos(currentElevationAngleDegrees) - radialVelocityMPS);
    
    double calculatedElevationAngleDegrees = Math.atan(verticalElevationVelocityMPS / (SHOOTER_SPEED_MPS * (Math.cos(currentElevationAngleDegrees))));
    double deltaElevationAngleDegrees = currentElevationAngleDegrees - calculatedElevationAngleDegrees;

    Pair<Double, Double> calculatedDeltaAngleGivenSpeeds = new Pair<Double, Double>(deltaAzimuthAngleDegrees, deltaElevationAngleDegrees);


    return calculatedDeltaAngleGivenSpeeds;


  }
}

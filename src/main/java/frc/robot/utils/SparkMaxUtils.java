package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import java.util.function.BooleanSupplier;

/** Mostly borrowed from 3005 */
public class SparkMaxUtils {

  /**
   * @param error API return value
   * @return
   */
  public static int check(REVLibError error) {
    return error == REVLibError.kOk ? 0 : 1;
  }

  public static class UnitConversions {

    /** Sets the encoder to read degrees after some gear ratio. */
    public static REVLibError setDegreesFromGearRatio(
        AbsoluteEncoder sparkMaxEncoder, double ratio) {
      double degreesPerRotation = 360.0 / ratio;
      double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
    }

    /** Sets the encoder to read radians after some gear ratio. */
    public static REVLibError setRadsFromGearRatio(AbsoluteEncoder sparkMaxEncoder, double ratio) {
      double radsPerRotation = (2.0 * Math.PI) / ratio;
      double radsPerRotationPerSecond = radsPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
    }

    /** Sets the encoder to read a linear distance with a gear ratio and a conversion to linear.
     * @param sparkMaxEncoder The encoder to set on.
     * @param ratio The gear ratio from motor to the winch (or other rotary-to-linear converter). >1 is a reduction.
     * @param diameter The diameter of the winch (or other rotary-to-linear converter).
     */
    public static REVLibError setLinearFromGearRatio(
        AbsoluteEncoder sparkMaxEncoder, final double ratio, final double diameter) {
      double linearDistPerRotation = diameter * Math.PI / ratio;
      double linearDistPerRotationPerSecond = linearDistPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(linearDistPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(linearDistPerRotationPerSecond);
    }

    /** Sets the encoder to read degrees after some gear ratio. */
    public static REVLibError setDegreesFromGearRatio(
        RelativeEncoder sparkMaxEncoder, double ratio) {
      double degreesPerRotation = 360.0 / ratio;
      double degreesPerRotationPerSecond = degreesPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(degreesPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(degreesPerRotationPerSecond);
    }

    /** Sets the encoder to read radians after some gear ratio. */
    public static REVLibError setRadsFromGearRatio(RelativeEncoder sparkMaxEncoder, double ratio) {
      double radsPerRotation = (2.0 * Math.PI) / ratio;
      double radsPerRotationPerSecond = radsPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(radsPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(radsPerRotationPerSecond);
    }

    /** Sets the encoder to read a linear distance with a gear ratio and a conversion to linear.
     * @param sparkMaxEncoder The encoder to set on.
     * @param ratio The gear ratio from motor to the winch (or other rotary-to-linear converter). >1 is a reduction.
     * @param diameter The diameter of the winch (or other rotary-to-linear converter).
     */
    public static REVLibError setLinearFromGearRatio(
        RelativeEncoder sparkMaxEncoder, final double ratio, final double diameter) {
      double linearDistPerRotation = diameter * Math.PI / ratio;
      double linearDistPerRotationPerSecond = linearDistPerRotation / 60.0;
      REVLibError error = sparkMaxEncoder.setPositionConversionFactor(linearDistPerRotation);

      if (error != REVLibError.kOk) {
        return error;
      }

      return sparkMaxEncoder.setVelocityConversionFactor(linearDistPerRotationPerSecond);
    }
  }

  public static String faultWordToString(short faults) {
    if (faults == 0) {
      return "";
    }

    StringBuilder builder = new StringBuilder();
    int faultsInt = faults;
    for (int i = 0; i < 16; i++) {
      if (((1 << i) & faultsInt) != 0) {
        builder.append(CANSparkMax.FaultID.fromId(i).toString());
        builder.append(" ");
      }
    }
    return builder.toString();
  }

  /**
   * Takes a function returning true on success, and tries running it until it succeeds up to the
   * max retry attempts
   */
  public static void initWithRetry(BooleanSupplier initFunction, int maxRetryAttempts) {
    int numAttempts = 0;
    while (!initFunction.getAsBoolean()) {
      numAttempts++;
      if (numAttempts > maxRetryAttempts) {
        break;
      }
    }
  }
}

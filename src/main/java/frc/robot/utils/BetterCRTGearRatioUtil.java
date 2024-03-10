package frc.robot.utils;

import java.util.LinkedList;
import java.util.Optional;

public class BetterCRTGearRatioUtil {
  private final double mainGearRotationRatioTerm;
  private final double mainGearRotationRatioTermSimplified;
  private final double secondaryGearRotationRatioTerm;
  private final double secondaryGearRotationRatioTermSimplfied;
  private final int maxRotations;
  private final double mainGearCircumference;
  private final double k;
  private LinkedList<Double> expectedAbsoluteEncoderValueMain = new LinkedList<Double>();
  private LinkedList<Double> expectedAbsoluteEncoderValueSecondary = new LinkedList<Double>();
  private double expectedAbsoluteEncoderValuePairsPrecisionDenominator = 30;

  public static Optional<BetterCRTGearRatioUtil> init(
      int mainGearRotationRatioTerm,
      int secondaryGearRotationRatioTerm,
      int maxRotations,
      double mainGearCircumference) {
    BetterCRTGearRatioUtil u =
        new BetterCRTGearRatioUtil(
            mainGearRotationRatioTerm,
            secondaryGearRotationRatioTerm,
            maxRotations,
            mainGearCircumference);
    if (u.validateGearRatioWithMaxRotations()) {
      // need to fill in expected values
      u.initExpectedAEVP();
      return Optional.of(u);
    }
    return Optional.empty();
  }

  private void initExpectedAEVP() {
    for (int i = 0; i < maxRotations * expectedAbsoluteEncoderValuePairsPrecisionDenominator; i++) {
      double elevatorPos =
          i * mainGearCircumference / expectedAbsoluteEncoderValuePairsPrecisionDenominator;
      double mainGearRotationValue =
          BetterCRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) > 0.98
              ? 0.0
              : BetterCRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) < 0.02
                  ? 0.0
                  : BetterCRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1);
      double secondaryGearRotationValue =
          BetterCRTGearRatioUtil.mod(
              elevatorPos
                  / mainGearCircumference
                  * secondaryGearRotationRatioTerm
                  / mainGearRotationRatioTerm,
              1);
      boolean has = false;
      for (int j = 0; j < expectedAbsoluteEncoderValueMain.size(); j++) {
        if (Math.abs(mainGearRotationValue - expectedAbsoluteEncoderValueMain.get(j)) < 0.005) {
          has = true;
        }
      }
      if (!has) {
        expectedAbsoluteEncoderValueMain.add(mainGearRotationValue);
      }
      has = false;
      for (int j = 0; j < expectedAbsoluteEncoderValueSecondary.size(); j++) {
        if (Math.abs(secondaryGearRotationValue - expectedAbsoluteEncoderValueSecondary.get(j))
            < 0.005) {
          has = true;
        }
      }
      if (!has) {
        expectedAbsoluteEncoderValueSecondary.add(secondaryGearRotationValue);
      }
    }
  }

  public double getAbsolutePositionOfMainObject(
      double mainGearRotationValue, double secondaryGearRotationValue) {
    double leastErrorMain = 1;
    double leastErrorSecondary = 1;
    double main = 0;
    double secondary = 0;
    for (int i = 0; i < expectedAbsoluteEncoderValueMain.size(); i++) {
      if (Math.abs(expectedAbsoluteEncoderValueMain.get(i) - mainGearRotationValue)
          < leastErrorMain) {
        main = expectedAbsoluteEncoderValueMain.get(i);
        leastErrorMain = Math.abs(expectedAbsoluteEncoderValueMain.get(i) - mainGearRotationValue);
      }
    }
    for (int i = 0; i < expectedAbsoluteEncoderValueSecondary.size(); i++) {
      if (Math.abs(expectedAbsoluteEncoderValueSecondary.get(i) - secondaryGearRotationValue)
          < leastErrorSecondary) {
        secondary = expectedAbsoluteEncoderValueSecondary.get(i);
        leastErrorSecondary =
            Math.abs(expectedAbsoluteEncoderValueSecondary.get(i) - secondaryGearRotationValue);
      }
    }
    System.out.println(main + "-" + secondary);
    return getAbsolutePositionOfMainObjectHelper(main, secondary);
  }

  public int getMainGearFullRotations(
      double mainGearRotationValue, double secondaryGearRotationValue) {
    return (int)
        Math.round(
            mod(
                mainGearRotationRatioTerm, // mod the main gear rotation ratio term by the whole
                // thing plus the simplified term to make 0 full
                // rotations work properly
                mod(
                        k
                            * // this is the multiplier, see the comment in the returns statement of
                            // kFactor() for more details
                            (mainGearRotationValue * secondaryGearRotationRatioTermSimplfied
                                - secondaryGearRotationValue
                                    * mainGearRotationRatioTermSimplified) // this returns a decimal
                        // where the multiplier
                        // is different for every
                        // gear ratio
                        ,
                        mainGearRotationRatioTermSimplified) // mod it with the main gear rotation
                    // ratio term so it is within the
                    // correct range
                    + mainGearRotationRatioTermSimplified)); // for more information, see the
    // comment in the kFactor() function
  }

  private double getAbsolutePositionOfMainObjectHelper(
      double mainGearRotationValue, double secondaryGearRotationValue) {
    return mainGearRotationValue * mainGearCircumference
        + getMainGearFullRotations(mainGearRotationValue, secondaryGearRotationValue)
            * mainGearCircumference;
  }

  private BetterCRTGearRatioUtil(
      int mainGearRotationRatioTerm,
      int secondaryGearRotationRatioTerm,
      int maxRotations,
      double mainGearCircumference) {
    this.mainGearRotationRatioTerm = (double) mainGearRotationRatioTerm;
    this.secondaryGearRotationRatioTerm = (double) secondaryGearRotationRatioTerm;
    this.mainGearRotationRatioTermSimplified =
        (double)
            (mainGearRotationRatioTerm
                / gcd(mainGearRotationRatioTerm, secondaryGearRotationRatioTerm));
    this.secondaryGearRotationRatioTermSimplfied =
        (double)
            (secondaryGearRotationRatioTerm
                / gcd(mainGearRotationRatioTerm, secondaryGearRotationRatioTerm));
    this.maxRotations = maxRotations;
    this.mainGearCircumference = mainGearCircumference;
    this.k = kFactor();
  }

  private static double gcd(double a, double b) {
    if (b == 0.0) return a;
    return gcd(b, a % b);
  }

  public static double mod(double a, double b) {
    double r = a % b;
    if (r < 0) {
      r += b;
    }
    return r;
  }

  private int kFactor() {
    double sum = 0;
    double a = secondaryGearRotationRatioTermSimplfied;
    double b = mainGearRotationRatioTermSimplified;
    for (int i = 1; i < b; i++) {
      double x = ((a * (b - i) - 1.0) / b);
      sum += x * (1 + Math.floor(x) - Math.ceil(x));
    }
    return (int) ((int) ((sum * b) + 1.0) / a);
  }

  public boolean validateGearRatioWithMaxRotations() {
    return maxRotations * mainGearCircumference < wrapAroundAbsolutePosition();
  }

  public double wrapAroundAbsolutePosition() {
    return mainGearCircumference * mainGearRotationRatioTerm;
  }
}

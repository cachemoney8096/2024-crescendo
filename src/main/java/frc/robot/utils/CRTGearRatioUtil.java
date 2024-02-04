package frc.robot.utils;

import java.util.Optional;

/* Helpful CRT functions - calculate the absolute position of an object attached to a gear
 * 
 * How to use
 * -Initialize the crtutil using the init function
 * -Then, to get the absolute location of the object attached to the gear, 
 * call getAbsolutePositionOfMainObject on the util object with the values from the two gear
 * 
 */
public class CRTGearRatioUtil {
  private final double mainGearRotationRatioTerm;
  private final double mainGearRotationRatioTermSimplified;
  private final double secondaryGearRotationRatioTermSimplfied;
  private final double secondaryGearRotationRatioTerm;
  private final int maxRotations;
  private final double mainGearCircumference;

  /**
   * 
   * @param mainGearRotationRatioTerm      for every mainGearRotationRatioTerm
   *                                       rotations of the main gear,
   *                                       the secondary gear rotates
   *                                       secondaryGearRotationRatioTerm times
   * @param secondaryGearRotationRatioTerm for every mainGearRotationRatioTerm
   *                                       rotations of the main gear,
   *                                       the secondary gear rotates
   *                                       secondaryGearRotationRatioTerm times
   * @param maxRotations                   the maximum number of rotations that
   *                                       the main gear will make that involve
   *                                       unique positions of the object
   *                                       attached to it
   * @param mainGearCircumference          circumference of the main gear
   * 
   * @return Returns optional of the util if the gear ratio works with the number
   *         of max rotations, otherwise returns an empty optional
   */
  public static Optional<CRTGearRatioUtil> init(int mainGearRotationRatioTerm, int secondaryGearRotationRatioTerm,
      int maxRotations, double mainGearCircumference) {
    CRTGearRatioUtil u = new CRTGearRatioUtil(mainGearRotationRatioTerm, secondaryGearRotationRatioTerm, maxRotations,
        mainGearCircumference);
    if (u.validateGearRatioWithMaxRotations()) {
      return Optional.of(u);
    }
    return Optional.empty();
  }

  private CRTGearRatioUtil(int mainGearRotationRatioTerm, int secondaryGearRotationRatioTerm, int maxRotations,
      double mainGearCircumference) {
    this.mainGearRotationRatioTerm = (double) mainGearRotationRatioTerm;
    this.secondaryGearRotationRatioTerm = (double) secondaryGearRotationRatioTerm;
    this.mainGearRotationRatioTermSimplified = (double) (mainGearRotationRatioTerm
        / gcd(mainGearRotationRatioTerm, secondaryGearRotationRatioTerm));
    this.secondaryGearRotationRatioTermSimplfied = (double) (secondaryGearRotationRatioTerm
        / gcd(mainGearRotationRatioTerm, secondaryGearRotationRatioTerm));
    this.maxRotations = maxRotations;
    this.mainGearCircumference = mainGearCircumference;
  }

  public static double gcd(double a, double b) {
    if (b == 0.0)
      return a;
    return gcd(b, a % b);
  }

  public static double mod(double a, double b) {
    double r = a % b;
    if (r < 0) {
      r += b;
    }
    return r;
  }

  /**
   * 
   * @return Returns a multiplier used by CRT. You shouldn't need to use this, and
   *         I don't fully understand it (Victor Chen did the math behind it)
   */
  public int kFactor() {
    double sum = 0;
    double a = secondaryGearRotationRatioTermSimplfied;
    double b = mainGearRotationRatioTermSimplified;
    for (int i = 1; i < b; i++) {
      sum += ((a * (b - i) - 1.0) / b)
          * (1 + Math.floor(((a * (b - i) - 1.0) / b)) - Math.ceil(((a * (b - i) - 1.0) / b)));
    }
    return (int) ((int) ((sum * b) + 1.0) / a);
  }

  /**
   * 
   * @param mainGearRotationValue      Rotation value returned by the main gear
   *                                   encoder
   * @param secondaryGearRotationValue Rotation value returned by the secondary
   *                                   gear encoder
   * @return Returns the number of full rotations of the main gear
   */
  public int getMainGearFullRotations(double mainGearRotationValue, double secondaryGearRotationValue) {
    double k = kFactor();
    return (int) Math.round(mod(mainGearRotationRatioTerm,
        mod(k * (mainGearRotationValue * secondaryGearRotationRatioTermSimplfied
            - secondaryGearRotationValue * mainGearRotationRatioTermSimplified), mainGearRotationRatioTermSimplified)
            + mainGearRotationRatioTermSimplified));
  }

  /**
   * 
   * @return Returns true if the gear ratio works with the neccessary number of
   *         max rotations, false otherwise
   */
  public boolean validateGearRatioWithMaxRotations() {
    boolean works = true;
    for (int i = 0; i < maxRotations * 6; i++) {
      double elevatorPos = i * mainGearCircumference / 6;
      // System.out.println(elevatorPos);
      double epoc = CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) > 0.9 ? 0.0
          : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) < 0.1 ? 0.0
              : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1);
      if (Math.abs(this.getAbsolutePositionOfMainObject(epoc,
          CRTGearRatioUtil.mod(
              elevatorPos / mainGearCircumference * secondaryGearRotationRatioTerm /
                  mainGearRotationRatioTerm,
              1))
          - elevatorPos) > 0.01) {
        works = false;
      }
    }
    return works;
  }

  /**
   * 
   * @param mainGearRotationValue
   * @param secondaryGearRotationValue
   * @return Applies CRT to find the absolute position of the object attached to
   *         the main gear
   */
  public double getAbsolutePositionOfMainObject(double mainGearRotationValue, double secondaryGearRotationValue) {
    return mainGearRotationValue * mainGearCircumference
        + getMainGearFullRotations(mainGearRotationValue, secondaryGearRotationValue) * mainGearCircumference;
  }

  /**
   * @return Returns the absolute position of the main object before it wraps around
   */
  public double wrapAroundAbsolutePosition(){
    return mainGearCircumference*maxRotations;
  }

}

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
    if (u.validateGearRatioWithMaxRotations()) { //check if the gear ratio works with the required amount of rotations by the main gear
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

  /**
   * 
   * @param a
   * @param b
   * @return Greatest common divisor of a and b
   */
  private static double gcd(double a, double b) {
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
   * @return Returns a multiplier used by CRT. You shouldn't need to use this function yourself, and
   *         I don't fully understand it (Victor Chen did the math behind it).
   */
  private int kFactor() {
    double sum = 0;
    double a = secondaryGearRotationRatioTermSimplfied;
    double b = mainGearRotationRatioTermSimplified;
    for (int i = 1; i < b; i++) {
      double x = ((a * (b - i) - 1.0) / b);
      sum += x
          * (1 + Math.floor(x) - Math.ceil(x));
    }
    return (int) ((int) ((sum * b) + 1.0) / a); //basically, when you apply CRT, you can multiply by a certain factor (what this function returns, a constant for a given gear ratio) such that when using modding that factor times the output of crt and the simplified main gear rotation ratio term, it will correctly output a number between 0 and maxrotations. then, to account for wrap around and to make 0 full rotations work correctly, mod the main gear ratio term by the output + the simplified term
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
    return (int) Math.round(mod(mainGearRotationRatioTerm, //mod the main gear rotation ratio term by the whole thing plus the simplified term to make 0 full rotations work properly
        mod(k * //this is the multiplier, see the comment in the returns statement of kFactor() for more details
        (mainGearRotationValue * secondaryGearRotationRatioTermSimplfied
            - secondaryGearRotationValue * mainGearRotationRatioTermSimplified) //this returns a decimal where the multiplier is different for every gear ratio
            , mainGearRotationRatioTermSimplified) //mod it with the main gear rotation ratio term so it is within the correct range
            + mainGearRotationRatioTermSimplified)); //for more information, see the comment in the kFactor() function
  }

  /**
   * 
   * @return Returns true if the gear ratio works with the neccessary number of
   *         max rotations, false otherwise
   */
  public boolean validateGearRatioWithMaxRotations() {
    boolean works = true;
    for (int i = 0; i < maxRotations * 6; i++) {
      double elevatorPos = i * mainGearCircumference / 6; //set elevator positon (expected position that it will check with) through sixths of a rotation until max rotations is reached
      double epoc = CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) > 0.9 ? 0.0
          : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) < 0.1 ? 0.0
              : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1); //calculate what the main gear encoder value would be at each position - the ternary operators account for annoying float division rounding problems on either end (0 and 1)
      if (Math.abs(this.getAbsolutePositionOfMainObject(epoc, //the calculated main gear encoder value
          CRTGearRatioUtil.mod(
              elevatorPos / mainGearCircumference * secondaryGearRotationRatioTerm /
                  mainGearRotationRatioTerm, //the calculated secondary gear encoder value
              1))
          - elevatorPos) > 0.01) { //check if the difference in what the function returns is greater than a small error factor, if it is, set working to false - this is a result more annoying float division problems, but will also not work if the gear ratio doesn't work
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
        + getMainGearFullRotations(mainGearRotationValue, secondaryGearRotationValue) * mainGearCircumference; //multiplies the number of full rotations plus the fractional amount of rotation from the main gear encoder by the circumference to determine distance traveled by the object attached to the main gear
  }

  /**
   * @return Returns the absolute position of the main object after one wrap around (my brain is cooked right now and the logic isn't logicing, somebody check this please)
   */
  public double wrapAroundAbsolutePosition(){
    return mainGearCircumference*mainGearRotationRatioTermSimplified;
  }

}

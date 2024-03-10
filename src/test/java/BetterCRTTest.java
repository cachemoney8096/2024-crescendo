import frc.robot.utils.BetterCRTGearRatioUtil;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class BetterCRTTest {
  @Test
  void test() {
    // we know which of these should work based on the wrap around point compared to the maximum
    // number of rotations we need to do
    /*assert checker(44, 42, 22);
    assert checker(44, 26, 22);
    assert checker(26, 44, 12);
    assert checker(42, 44, 20);
    assert !checker(3, 4, 22);
    assert !checker(4, 3, 10);
    assert checker(28, 30, 14);*/
    assert checker(44, 26, 11);
  }
  /**
   * @param mgr Main gear rotation ratio term
   * @param sgr Secondary gear rotation ratio term
   * @param mr Max rotations
   * @return True if the gear ratio works, false otherwise
   */
  boolean checker(int mgr, int sgr, int mr) {
    int mainGearRotationRatioTerm = mgr;
    int secondaryGearRotationRatioTerm = sgr;
    int maxRotations = mr;
    double mainGearCircumference = 1.1 * Math.PI;
    Optional<BetterCRTGearRatioUtil> crtUtilOptional =
        BetterCRTGearRatioUtil.init(
            mainGearRotationRatioTerm,
            secondaryGearRotationRatioTerm,
            maxRotations,
            mainGearCircumference);
    if (!crtUtilOptional.isPresent()) {
      return false;
    }
    BetterCRTGearRatioUtil crtutil = crtUtilOptional.get();
    boolean works = true;
    for (int i = 0; i < maxRotations * 6; i++) {
      double elevatorPos = i * mainGearCircumference / 6;
      double epoc = // elevator pos / cir
          BetterCRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) > 0.98
              ? 0.0
              : BetterCRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) < 0.02
                  ? 0.0
                  : BetterCRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1);
      if (Math.abs(
              crtutil.getAbsolutePositionOfMainObject(
                      epoc,
                      BetterCRTGearRatioUtil.mod(
                          elevatorPos
                              / mainGearCircumference
                              * secondaryGearRotationRatioTerm
                              / mainGearRotationRatioTerm,
                          1))
                  - elevatorPos)
          > 1) {
        works = false;
      }
      System.out.println(
          crtutil.getAbsolutePositionOfMainObject(
              epoc,
              BetterCRTGearRatioUtil.mod(
                  elevatorPos
                      / mainGearCircumference
                      * secondaryGearRotationRatioTerm
                      / mainGearRotationRatioTerm,
                  1)));
    }
    System.out.println("");
    System.out.println(crtutil.getAbsolutePositionOfMainObject(0.7, 0.7 * 26 / 44));
    System.out.println(crtutil.getAbsolutePositionOfMainObject(0.7, (0.7 * 26 / 44) + 0.004));
    return works;
  }
}

import frc.robot.utils.CRTGearRatioUtil;
import org.junit.jupiter.api.Test;
import java.util.Optional;

public class CRTGearRatioUtilTestingStuff {
  @Test
  void test() {
    assert checker(44, 42, 22);
    assert checker(44, 26, 22);
    assert !checker(3, 4, 22);
    assert !checker(4,3,10);
    assert checker(28,30,14);
  }
  /*Main gear ratio, secondary gear ratio, max rotations */
  boolean checker(int mgr, int sgr, int mr){
    int mainGearRotationRatioTerm = mgr;
    int secondaryGearRotationRatioTerm = sgr;
    int maxRotations = mr;
    double mainGearCircumference = Math.PI;
    Optional<CRTGearRatioUtil> crtutilOptional = CRTGearRatioUtil.init(mainGearRotationRatioTerm, secondaryGearRotationRatioTerm, maxRotations,
        mainGearCircumference);
    if(!crtutilOptional.isPresent()){
      return false;
    }
    CRTGearRatioUtil crtutil = crtutilOptional.get();
    boolean works = true;
     for (int i = 0; i < maxRotations * 6; i++) {
     double elevatorPos = i * mainGearCircumference / 6;
     double epoc = CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) >
     0.9 ? 0.0
     : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) < 0.1 ? 0.0
     : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1);
     if(Math.abs(crtutil.getAbsolutePositionOfMainObject(epoc,
      CRTGearRatioUtil.mod(
      elevatorPos / mainGearCircumference * secondaryGearRotationRatioTerm /
      mainGearRotationRatioTerm,
      1)) - elevatorPos) > 0.01){
        works = false;
      }
     }
     return works;
  }
}

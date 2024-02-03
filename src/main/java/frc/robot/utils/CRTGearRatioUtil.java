package frc.robot.utils;

import java.util.Arrays;

/*Helpful CRT functions */
public class CRTGearRatioUtil {
  private final double mainGearRotationRatioTerm;
  private final double mainGearRotationRatioTermSimplified;
  private final double secondaryGearRotationRatioTermSimplfied;
  private final double secondaryGearRotationRatioTerm;
  private final int maxRotations;
  private final double mainGearCircumference;

  public CRTGearRatioUtil(int mainGearRotationRatioTerm, int secondaryGearRotationRatioTerm, int maxRotations, double mainGearCircumference) {
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

  // =(mod(GRD, mod(K*(E1*SGRN-E2*SGRD), SGRD)+SGRD))*PI()+E1*PI()
  // denominator=main gear
  public int getMainGearFullRotations(double mainGearRotationValue, double secondaryGearRotationValue) {
    double k = kFactor();
    return (int) Math.round(mod(mainGearRotationRatioTerm,
        mod(k * (mainGearRotationValue * secondaryGearRotationRatioTermSimplfied
            - secondaryGearRotationValue * mainGearRotationRatioTermSimplified), mainGearRotationRatioTermSimplified)
            + mainGearRotationRatioTermSimplified));
  }

  public boolean validateGearRatioWithMaxRotations(){
    boolean works = true;
     for (int i = 0; i < maxRotations * 6; i++) {
     double elevatorPos = i * mainGearCircumference / 6;
     // System.out.println(elevatorPos);
     double epoc = CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) >
     0.9 ? 0.0
     : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1) < 0.1 ? 0.0
     : CRTGearRatioUtil.mod(elevatorPos / mainGearCircumference, 1);
     if(Math.abs(this.getAbsolutePositionOfMainObject(epoc,
      CRTGearRatioUtil.mod(
      elevatorPos / mainGearCircumference * secondaryGearRotationRatioTerm /
      mainGearRotationRatioTerm,
      1)) - elevatorPos) > 0.01){
        works = false;
      }
     }
    return works;
  }

  public double getAbsolutePositionOfMainObject(double mainGearRotationValue, double secondaryGearRotationValue){
    return mainGearRotationValue*Math.PI+getMainGearFullRotations(mainGearRotationValue, secondaryGearRotationValue)*Math.PI;
  }

}

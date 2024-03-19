package frc.robot.utils;

import java.util.Optional;
import java.util.LinkedList;
import edu.wpi.first.math.Pair;

public class CRTUtil {
  private final double mainGearRatioTerm;
  private final double secondaryGearRatioTerm;
  private final double mainGearCircumference;
  private final double requiredMaxRotations;
  private final double mainGearRatioTermSimplified;
  private final double secondaryGearRatioTermSimplfied;
  private LinkedList<Pair<Pair<Double, Double>,Double>> table = new LinkedList<>();

  private CRTUtil(double mainGearRatioTerm, double secondaryGearRatioTerm, double mainGearCircumference, int requiredMaxRotations){
    this.mainGearRatioTerm = mainGearRatioTerm;
    this.secondaryGearRatioTerm = secondaryGearRatioTerm;
    this.mainGearCircumference = mainGearCircumference;
    this.requiredMaxRotations = requiredMaxRotations;
    this.mainGearRatioTermSimplified =
        (double)
            (mainGearRatioTerm
                / gcd(mainGearRatioTerm, secondaryGearRatioTerm));
    this.secondaryGearRatioTermSimplfied =
        (double)
            (secondaryGearRatioTerm
                / gcd(mainGearRatioTerm, secondaryGearRatioTerm));
  }

  public static Optional<CRTUtil> init(double mainGearRatioTerm, double secondaryGearRatioTerm, double mainGearCircumference, int requiredMaxRotations, double allowedError){
    if(allowedError <= 0){
      return Optional.empty();
    }
    CRTUtil crtUtil = new CRTUtil(mainGearRatioTerm, secondaryGearRatioTerm, mainGearCircumference, requiredMaxRotations);
    //double fractionalPrecisionDenominator = Math.ceil(mainGearCircumference/allowedError);
    double fractionalPrecisionDenominator = 6;
    crtUtil.initTable(fractionalPrecisionDenominator);
    if(!crtUtil.validateTable()){
      return Optional.empty();
    }
    return Optional.of(crtUtil);
  }

  private void initTable(double fractionalPrecisionDenominator){
    for(int i = 0; i < requiredMaxRotations*fractionalPrecisionDenominator; i++){
      double elevatorPosition = (i/fractionalPrecisionDenominator)*mainGearCircumference;
      double encoderOne = mod(elevatorPosition/mainGearCircumference, 1);
      double encoderTwo = mod((elevatorPosition/mainGearCircumference)*(mainGearRatioTerm/secondaryGearRatioTerm), 1);
      table.add(new Pair<Pair<Double, Double>, Double>(new Pair<Double, Double>(encoderOne, encoderTwo), elevatorPosition));
    }
  }

  private boolean validateTable(){
    double last = table.get(0).getSecond();
    for(int i = 1; i < table.size(); i++){
      if(table.get(i).getSecond() < last){
        return false;
      }
    }
    return true;
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

  /*private int kFactor() {
    double sum = 0;
    double a = secondaryGearRatioTermSimplfied;
    double b = mainGearRatioTermSimplified;
    for (int i = 1; i < b; i++) {
      double x = ((a * (b - i) - 1.0) / b);
      sum += x * (1 + Math.floor(x) - Math.ceil(x));
    }
    return (int)
        ((int) ((sum * b) + 1.0)
            / a); 
  }

  private double absolutePosition(double encoderOne, double encoderTwo){
    return (mod(mainGearRatioTerm, mod(kFactor()*(encoderOne*mainGearRatioTermSimplified-encoderTwo*secondaryGearRatioTermSimplfied), secondaryGearRatioTermSimplfied)+secondaryGearRatioTermSimplfied))*mainGearCircumference+encoderOne*mainGearCircumference;
  }*/

  public double getAbsolutePosition(double mainEncoder, double secondaryEncoder){
    Pair<Pair<Double, Double>, Double> fixedPair = table.get(0);
    Pair<Pair<Double, Double>, Double> inputPair = new Pair<Pair<Double, Double>, Double>(new Pair<Double, Double>(mainEncoder, secondaryEncoder), null);
    for(int i = 1; i < table.size(); i++){
      if(getPositionPairErrorMargin(inputPair, table.get(i)) < getPositionPairErrorMargin(fixedPair, inputPair)){
        fixedPair = table.get(i);
      }
    }
    return fixedPair.getSecond();
  }

  private double getPositionPairErrorMargin(Pair<Pair<Double, Double>, Double> one, Pair<Pair<Double, Double>, Double> two){
    return Math.abs(one.getFirst().getFirst()-two.getFirst().getFirst()) + Math.abs(one.getFirst().getSecond()-two.getFirst().getSecond());
  }
}

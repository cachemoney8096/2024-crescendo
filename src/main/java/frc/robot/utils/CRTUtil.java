package frc.robot.utils;

public class CRTUtil{

    private int mainGearRatioTerm;
    private int secondaryGearRatioTerm;

    /* Gear ratio terms for main and secondary gears*/
    public CRTUtil(int m, int s){
        mainGearRatioTerm = m;
        secondaryGearRatioTerm = s;
    }
    
    /* Extended euclidean algorithm https://en.wikipedia.org/wiki/Extended_Euclidean_algorithm */
    private int[] eea(int r0, int r1, int s0, int s1, int t0, int t1){
        int q = (int)Math.floor((double)r0/(double)r1);
        int r2 = r0 - q*r1;
        int s2 = s0 - q*s1;
        int t2 = t0 - q*t1;
        if(r2 == 0){
            int[] nums = {s1,t1};
            return nums;
        }
        else{
            return eea(r1, r2, s1, s2, t1, t2);
        }
    }

    /* Chinese remainder theorem https://en.wikipedia.org/wiki/Chinese_remainder_theorem */
    private int crt(int n1, int n2, int a1, int a2){ 
        int[] ms = eea(n1, n2, 1, 0, 0, 1);
        int m1 = ms[0];
        int m2 = ms[1];
        int result = a2*m1*n1+a1*m2*n2;
        System.out.println("Unchanged res: " + result);
        result = (result%(n1*n2)+(n1*n2))%(n1*n2);
        return result;
    }

    /* Rotation fraction to current cycle numerator*/
    private int rftccn(double fraction, int denominator){
        return (int)(Math.round(fraction*(double)denominator));
    }

    /* Full rotations of each gear [mainGear, secondaryGear] */
    public int[] fullRotations(double mainGearEncoderValue, double secondaryGearEncoderValue){
        int crtResult = crt(mainGearRatioTerm, secondaryGearRatioTerm, rftccn(mainGearEncoderValue, mainGearRatioTerm), rftccn(secondaryGearEncoderValue, secondaryGearRatioTerm));
        int[] result = {(int)(Math.floor((double)crtResult/(double)mainGearRatioTerm)), (int)(Math.floor((double)crtResult/(double)secondaryGearRatioTerm))};
        return result;
    }

    public static void unitTest() {
        double acceptableDrumRevolutionsError = 10;
        int passedTests = 0;
        int failedTests = 0;

        CRTUtil crt = new CRTUtil(14, 15);
        // double drumRadius = 1.0;
        // double drumRevolutions;
        double mainEncoderPosition;
        double secondaryEncoderPosition;

        for (double drumRevolutions = 0.0; drumRevolutions < 100.0; drumRevolutions += 2.0) {
            // double drumCircumference = drumRadius * 2 * Math.PI;
            // double drumCircumference = drumRadius;
            // drumRevolutions = elevatorPosition / drumCircumference;
            mainEncoderPosition = (drumRevolutions - crt.mainGearRatioTerm * 
                Math.floor(drumRevolutions / crt.mainGearRatioTerm)) / crt.mainGearRatioTerm;
            secondaryEncoderPosition = (drumRevolutions - crt.secondaryGearRatioTerm * 
                Math.floor(drumRevolutions / crt.secondaryGearRatioTerm)) / crt.secondaryGearRatioTerm;
            int[] crtResult = crt.fullRotations(mainEncoderPosition, secondaryEncoderPosition);
            double crtDrumRevolutions = (double)crtResult[0] * (double)crt.mainGearRatioTerm;

            if (crtDrumRevolutions < drumRevolutions + acceptableDrumRevolutionsError &&
                crtDrumRevolutions > drumRevolutions - acceptableDrumRevolutionsError) {
                passedTests += 1;
            } else {
                failedTests += 1;
                System.out.println("-------------------");
                System.out.println("--- Failed test ---");
                System.out.println("-------------------");
                System.out.println("Actual Drum Revolutions " + drumRevolutions);
                System.out.println("Main Encoder " + mainEncoderPosition);
                System.out.println("Secondary Encoder " + secondaryEncoderPosition);
                System.out.println("CRT Rotations Main " + crtResult[0] + " Secondary " + crtResult[1]);
                System.out.println("CRT-calculated Drum Revolutions " + crtDrumRevolutions);
            }
        }
        System.out.println(passedTests + " tests passed :)");
        System.out.println(failedTests + " tests failed :(");
    }

    /* Test with values in sheet - plug in the gear ratio and encoder values https://docs.google.com/spreadsheets/d/11Y7OoW-knLFfJ6n38-v7QJLaetr21EX1DaB15RoD6Ug/edit?gid=103023517#gid=103023517*/
    // public static void main(String[] args){
    //     unitTest();
    //     // CRT crt = new CRT(14, 15);
    //     // System.out.println(crt.fullRotations(0.8333333333, 0.7777777778)[0]);
    // }
}
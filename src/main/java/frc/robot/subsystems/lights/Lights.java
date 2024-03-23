package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Lights {
  /**
   * Tree map of light code enums and integers that represent the R, G, B values of each light code
   * enum
   */
  private TreeMap<LightCode, Integer[]> lightOptionsMap;

  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;

  public enum LightCode {
    OFF, // nothing, used for prepState = OFF and UnclimbSequence
    INTAKING, // purple
    HAS_NOTE, // green
    SPEAKER_PREP, // blue
    FEED, // red
    AMP_PREP, // red, both feed are amp prep are red
    CLIMB_PREP, // yellow
    READY_TO_SCORE // rainbow
  }

  public Lights() {
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    lightOptionsMap = new TreeMap<LightCode, Integer[]>();
    lightOptionsMap.put(LightCode.OFF, new Integer[] {0, 0, 0});
    lightOptionsMap.put(LightCode.INTAKING, new Integer[] {255, 0, 255});
    lightOptionsMap.put(LightCode.HAS_NOTE, new Integer[] {0, 255, 0});
    lightOptionsMap.put(LightCode.SPEAKER_PREP, new Integer[] {0, 0, 255});
    lightOptionsMap.put(LightCode.FEED, new Integer[] {255, 0, 0});
    lightOptionsMap.put(LightCode.AMP_PREP, new Integer[] {255, 0, 0});
    lightOptionsMap.put(LightCode.CLIMB_PREP, new Integer[] {255, 255, 0});
  }

  public void setLEDColor(LightCode light) {
    currentLightStatus = light;
    setLEDs();
  }

  private void setLEDs() {
    if (currentLightStatus == LightCode.READY_TO_SCORE) {
      setRainbow();
    } else {
      candle.animate(null);
      candle.setLEDs(
          lightOptionsMap.get(currentLightStatus)[0],
          lightOptionsMap.get(currentLightStatus)[1],
          lightOptionsMap.get(currentLightStatus)[2]);
    }
  }

  private void setRainbow() {
    RainbowAnimation rainbowAnim =
        new RainbowAnimation(
            LightsConstants.LIGHT_BRIGHTNESS,
            LightsConstants.LIGHT_SPEED,
            LightsConstants.NUM_CANDLE_LEDS);
    candle.animate(rainbowAnim);
  }
}

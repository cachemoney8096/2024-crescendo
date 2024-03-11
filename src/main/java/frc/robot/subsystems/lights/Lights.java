package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Lights extends SubsystemBase{
  /**
   * Tree map of light code enums and integers that represent the R, G, B values of each light code
   * enum
   */
  private TreeMap<LightCode, Integer[]> lightOptionsMap;

  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;

  public enum LightCode {
    NOTELESS, // red
    INTAKING, // blue
    READY_TO_SHOOT, // green
    ALIGNING_TO_TAG, // orange
    HOLDING_NOTE, // yellow
    OFF, // no lights are on
    NO_TAG,
    PARTY_MODE;
  }

  public Lights() {
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    lightOptionsMap = new TreeMap<LightCode, Integer[]>();
    lightOptionsMap.put(LightCode.NOTELESS, new Integer[] {255, 0, 0});
    lightOptionsMap.put(LightCode.INTAKING, new Integer[] {0, 0, 255});
    lightOptionsMap.put(LightCode.READY_TO_SHOOT, new Integer[] {0, 255, 0});
    lightOptionsMap.put(LightCode.ALIGNING_TO_TAG, new Integer[] {255, 165, 0});
    lightOptionsMap.put(LightCode.HOLDING_NOTE, new Integer[] {255, 255, 102});
    lightOptionsMap.put(LightCode.OFF, new Integer[] {0, 0, 0});
  }

  public void toggleCode(LightCode light) {
    if (currentLightStatus == light) {
      currentLightStatus = LightCode.OFF;
    } else {
      currentLightStatus = light;
    }
    setLEDs();
  }

  private void setLEDs() {
    if (currentLightStatus == LightCode.PARTY_MODE) {
      setPartyMode();
    } else if (currentLightStatus == LightCode.NO_TAG) {
      setNoTag();
    } else {
      candle.setLEDs(
          lightOptionsMap.get(currentLightStatus)[0],
          lightOptionsMap.get(currentLightStatus)[1],
          lightOptionsMap.get(currentLightStatus)[2]);
    }
  }

  private void setNoTag() {
    StrobeAnimation strobeAnim = new StrobeAnimation(255, 0, 255);
    candle.animate(strobeAnim);
  }

  public void setPartyMode() {
    RainbowAnimation rainbowAnim =
        new RainbowAnimation(
            LightsConstants.LIGHT_BRIGHTNESS,
            LightsConstants.LIGHT_SPEED,
            LightsConstants.NUM_CANDLE_LEDS);
    candle.animate(rainbowAnim);
  }
}

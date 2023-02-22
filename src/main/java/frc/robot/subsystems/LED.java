package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(Constants.LED_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED_NUM_LEDS);
  private Color currentColor = null;
  private int startHue = 0;
  /** Creates a new LED. */
  public LED() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
  }
  // Turns LED on.
  public void enable(){
    led.start();
  }
  public void setPurple()
  {
    currentColor = new Color(58, 0, 181);
    for (int i = 0; i < buffer.getLength(); i++)
    {
      buffer.setLED(i, Color.kPurple);
    }
    led.setData(buffer);
  }
  public void setYellow()
  {
    currentColor = new Color(255, 100, 0);
    for (int i = 0; i < buffer.getLength(); i++)
    {
      buffer.setLED(i, currentColor);
    }
    led.setData(buffer);
  }
  public void rainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      final int hue = (startHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    led.setData(buffer);
    startHue += 3;
    startHue %= 180;
  }
  public void setPink() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, 178, 255, 255);
    }
    led.setData(buffer);
  }
  // Turns LED off.
  public void disable()
  {
    led.stop();
  }
  public Color getColor()
  {
    return currentColor;
  }
  @Override
  public void periodic() {}
}
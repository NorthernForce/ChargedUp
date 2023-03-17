package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.RobotContainer.manipulatingState;

public class LED extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(Constants.LEDConstants.PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDConstants.NUM_LEDS);
  private Color currentColor = null;
  private int startHue = 0;
  /** Creates a new LED. */
  public LED() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
  }
  /** Turns the led on */
  public void enable(){
    led.start();
  }
  /** Sets the led to the color purple: (127, 0, 127) */
  public void setPurple()
  {
    currentColor = new Color(58, 0, 181);
    for (int i = 0; i < buffer.getLength(); i++)
    {
      buffer.setLED(i, Color.kPurple);
    }
    led.setData(buffer);
  }
  /** Sets the led to the color yellow: (255, 100, 0) */
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
  public void setFromManipulatingState() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, manipulatingState.getCurrentState().getColor());
    }
    led.setData(buffer);
  }
  /** Turns LED off. */
  public void disable()
  {
    led.stop();
  }
  /**
   * Gets the current color displayed by the LED
   * @return wpilib Color object
   */
  public Color getColor()
  {
    return currentColor;
  }
  /** Overrides the subsytem's periodic */
  @Override
  public void periodic() {}
}
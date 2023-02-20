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
    currentColor = Color.kPurple;
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
      buffer.setLED(i, new Color(255, 100, 0));
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
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
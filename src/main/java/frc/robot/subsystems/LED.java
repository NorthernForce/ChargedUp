package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED_NUM_LEDS);
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
    for (int i = 0; i < buffer.getLength(); i++)
    {
      buffer.setLED(i, Color.kPurple);
    }
    led.setData(buffer);
    led.start();
  }
  public void setYellow()
  {
    for (int i = 0; i < buffer.getLength(); i++)
    {
      buffer.setLED(i, new Color(1, 1, 0));
    }
    led.setData(buffer);
    led.start();
  }
  // Turns LED off.
  public void disable(){
    led.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
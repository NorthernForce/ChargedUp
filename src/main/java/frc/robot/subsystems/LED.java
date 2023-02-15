package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.RobotContainer.activeChassis;

public class LED extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(activeChassis.getIntegerConstant("LED_PORT"));
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(activeChassis.getIntegerConstant("NUM_ACTIVE_LEDS"));
  private Color currentColor = null;
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
    currentColor = Color.kPurple;
    for (int i = 0; i < buffer.getLength(); i++)
    {
      buffer.setLED(i, Color.kPurple);
    }
    led.setData(buffer);
  }
  public void setYellow()
  {
    currentColor = Color.kYellow;
    for (int i = 0; i < buffer.getLength(); i++)
    {
      buffer.setLED(i, Color.kYellow);
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
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
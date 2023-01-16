// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * A Light Subsystem that controls a PWM led light that can be turned on or off, or set to any RGB
 */
public class LightSystem extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);
  /** Creates a new LightSystem. */
  public LightSystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
  }
  /**
   * Sets the state of the light
   * @param enabled whether the light is on or not
   */
  public void setState(boolean enabled)
  {
    if (enabled)
    {
      led.start();
    }
    else
    {
      led.stop();
    }
  }
  /**
   * Sets the RGB value of the light
   * @param r Red [0-255]
   * @param g Green [0-255]
   * @param b Blue [0-255]
   */
  public void setRGB(int r, int g, int b)
  {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
  }
  /**
   * Empty Implementation of the SubsystemBase periodic() method
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

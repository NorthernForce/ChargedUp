// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSystem extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);
  /** Creates a new LightSystem. */
  public LightSystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
  }

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

  public void setRGB(int r, int g, int b)
  {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

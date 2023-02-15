// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Relay;
import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber;
import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber;
import edu.wpi.first.wpilibj.util.Color;


public class LED extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(50);
  
  /** Creates a new LED. */
  public LED() {
    led.setLength(50);
    led.setData(buffer);
    putNumber("red", 255);
    putNumber("green", 100);
    putNumber("blue", 0);
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
      buffer.setLED(i, new Color(
        (int)getNumber("red", 255), 
        (int)getNumber("green", 100), 
        (int)getNumber("blue", 0)
      ));
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

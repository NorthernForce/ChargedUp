// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Relay;


public class LED extends SubsystemBase {
  private final Relay relay = new Relay(0);
  
  /** Creates a new LED. */
  public LED() {



  }
  // Turns LED on.
  public void enable(){
    relay.set(Relay.Value.kForward);
  
  }
  // Turns LED off.
  public void disable(){
    relay.set(Relay.Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

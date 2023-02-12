// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compressor extends SubsystemBase {
  /** Creates a new Compressor. */
  public Compressor() {}
  /** Enables Compressor */
  public void enable()
  {
  }
  /**
   * Sets solenoid to on or off
   * @param ID solenoid ID or port
   * @param state the state to set it at
   */
  public void setSolenoid(int ID, boolean state)
  {
  }
  /** Turns the compressor off */
  public void disable()
  {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

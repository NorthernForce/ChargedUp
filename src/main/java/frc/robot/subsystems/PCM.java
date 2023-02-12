// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PCM extends SubsystemBase {
  private final Compressor compressor = new Compressor(Constants.COMPRESSOR_ID,
    PneumaticsModuleType.REVPH);
  /** Creates a new Compressor. */
  public PCM() {}
  /**
   * Sets solenoid to on or off
   * @param ID solenoid ID or port
   * @param state the state to set it at
   */
  public void setSolenoid(int ID, boolean state)
  {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

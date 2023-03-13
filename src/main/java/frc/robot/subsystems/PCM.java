// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PCM extends SubsystemBase {
  private final Compressor compressor = new Compressor(Constants.CompressorConstants.COMPRESSOR_ID,
    PneumaticsModuleType.REVPH);
  private final Map<Integer, Solenoid> solenoids = new HashMap<>();
  /** Creates a new Compressor. */
  public PCM() {
    compressor.enableDigital();
  }
  /**
   * Sets state of the solenoid
   * @param ID solenoid ID
   * @param state the state to set it at
   */
  public void setSolenoidState(int ID, boolean state)
  {
    if (!solenoids.containsKey(ID))
    {
      solenoids.put(ID,
        new Solenoid(Constants.CompressorConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, ID)
      );
    }
    solenoids.get(ID).set(state);
  }
  /**
   * Gets the state of the solenoid
   * @param ID solenoid ID
   * @return the state it is at
   */
  public boolean getSolenoidState(int ID)
  {
    if (!solenoids.containsKey(ID))
    {
      solenoids.put(ID,
        new Solenoid(Constants.CompressorConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, ID)
      );
    }
    return solenoids.get(ID).get();
  }
  /**
   * Toggles the solenoid
   * @param ID solenoid ID
   */
  public void toggleSolenoid(int ID)
  {
    if (!solenoids.containsKey(ID))
    {
      solenoids.put(ID,
        new Solenoid(Constants.CompressorConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, ID)
      );
    }
    solenoids.get(ID).toggle();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.activeChassis;

public class PCM extends SubsystemBase {
  private final Compressor compressor = new Compressor(activeChassis.getIntegerConstant("COMPRESSOR_ID"),
    PneumaticsModuleType.REVPH);
  private final Map<Integer, DoubleSolenoid> solenoids = new HashMap<>();
  /** Creates a new Compressor. */
  public PCM() {
    compressor.enableDigital();
    solenoids.put(activeChassis.getIntegerConstant("MOTOR_SOLENOID_ID"),
      new DoubleSolenoid(PneumaticsModuleType.REVPH,
      activeChassis.getIntegerConstant("MOTOR_SOLENOID_FORWARD"), activeChassis.getIntegerConstant("MOTOR_SOLENOID_REVERSE"))
    );
  }
  /**
   * Sets state of the solenoid
   * @param ID solenoid ID
   * @param state the state to set it at
   */
  public void setSolenoidState(int ID, DoubleSolenoid.Value state)
  {
  }
  /**
   * Gets the state of the solenoid
   * @param ID solenoid ID
   * @return the state it is at
   */
  public DoubleSolenoid.Value getSolenoidState(int ID)
  {
    return null;
  }
  /**
   * Toggles the solenoid
   * @param ID solenoid ID
   */
  public void toggleSolenoid(int ID)
  {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

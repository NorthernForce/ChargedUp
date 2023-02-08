// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class Pneumatics extends SubsystemBase {
 // This creates all the diffrent solenoids used.
  private final DoubleSolenoid m_ArmSol =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  
  private final DoubleSolenoid m_Gripper1Sol =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  private final DoubleSolenoid m_Gripper2Sol =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  
  private final DoubleSolenoid m_WristSol =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);  
      
  private final DoubleSolenoid m_CoolingSol =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 8, 9);

  private final DoubleSolenoid m_Airbreak1Sol =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 10, 11); 

  private final DoubleSolenoid m_Airbreak2Sol =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 12, 13);    



@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

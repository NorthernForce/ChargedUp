// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class ArmTelescope extends SubsystemBase {
  private boolean isExtended = false;
  /** Creates a new ArmTelescope. */
  public ArmTelescope() {}
  public void extend()
  {
    isExtended = true;
    pcm.setSolenoidState(Constants.TELESCOPE_SOLENOID_ID, DoubleSolenoid.Value.kForward);
  }
  public void retract()
  {
    isExtended = false;
    pcm.setSolenoidState(Constants.TELESCOPE_SOLENOID_ID, DoubleSolenoid.Value.kReverse);
  }
  public boolean isExtended()
  {
    return isExtended;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

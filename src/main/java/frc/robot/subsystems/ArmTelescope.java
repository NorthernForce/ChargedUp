// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class ArmTelescope extends SubsystemBase {
  /** Creates a new ArmTelescope. */
  public ArmTelescope() {
    Shuffleboard.getTab("Arm").addBoolean("isExtended", () ->
      pcm.getSolenoidState(Constants.CompressorConstants.TELESCOPE_SOLENOID_ID)
    ).withPosition(1, 0);
  }
  public void extend()
  {
    pcm.setSolenoidState(Constants.CompressorConstants.TELESCOPE_SOLENOID_ID, true);
  }
  public void retract()
  {
    pcm.setSolenoidState(Constants.CompressorConstants.TELESCOPE_SOLENOID_ID, false);
  }
  public boolean isExtended()
  {
    return pcm.getSolenoidState(Constants.CompressorConstants.TELESCOPE_SOLENOID_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

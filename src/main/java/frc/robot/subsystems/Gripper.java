// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  public Gripper() {}
  public void clamp()
  {
  }
  public boolean hasObject()
  {
    return false;
  }
  public void release()
  {
  }
  public boolean isClamped()
  {
    return false;
  }
  public Rotation2d getWristAngle()
  {
    return null;
  }
  public void setWristAngle(Rotation2d rotation)
  {
  }
  public void setWristSpeed(double speed)
  {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

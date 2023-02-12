// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.*;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {
    // Assert required subsystem have been declared
    assert compressor != null;
  }
  /** Extends Arm */
  public void extend()
  {
  }
  /** Retracts Arm */
  public void retract()
  {
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return null;
  }
  /**
   * Set arm angle
   * @param angle angle to set the arm to
  */
  public void setAngle(Rotation2d angle)
  {
  }
  /**
   * Tells whether the arm is extended
   * @return whether the arm is extended
   */
  public boolean isExtended()
  {
    return false;
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

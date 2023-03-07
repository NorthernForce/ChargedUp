// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.variants.MotorGroupTalon;

import com.ctre.phoenix.sensors.CANCoder;

public class ArmRotate extends SubsystemBase {
  // We know we will have two talons
  private final MotorGroupTalon talonGroup;
  /** Creates a new Arm. */
  public ArmRotate() {
    talonGroup = new MotorGroupTalon(Constants.ARM_PRIMARY_MOTOR_ID, new int[]
    {
      Constants.ARM_FOLLOWER_MOTOR_ID
    });
    talonGroup.setFollowerOppose(0);
    talonGroup.configClosedLoop(0, 0, Constants.ARM_KF, Constants.ARM_KP, Constants.ARM_KI, Constants.ARM_KD);
    talonGroup.configSelectedProfile(0, 0);
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(talonGroup.getEncoderRotations());
  }
  /**
   * Set arm angle
   * @param angle angle to set the arm to
  */
  public void setAngle(Rotation2d angle)
  {
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
    SmartDashboard.putNumber("Arm Angle", getAngle().getDegrees());
  }
}
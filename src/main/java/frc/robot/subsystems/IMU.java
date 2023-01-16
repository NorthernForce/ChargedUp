// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A Subsystem responsible for maintaining the NavX on the RoboRIO.
 */
public class IMU extends SubsystemBase {
  private final AHRS ahrs = new AHRS();
  /** Creates a new IMU. */
  public IMU() {
    ahrs.reset();
  }
  /**
   * Gets the Rotation3d representing the rotation of the robot
   * @return Rotation3d - All angles are in Radians
   */
  public Rotation3d getRotation()
  {
    return new Rotation3d(Math.toRadians(ahrs.getPitch()), Math.toRadians(ahrs.getRoll()), Math.toRadians(ahrs.getYaw()));
  }
  /**
   * Gets the heading of the robot
   * @return A Rotation3d of the heading.
   */

  public double getYaw()
  {
    return ahrs.getYaw();
  }

  public double getRoll()
  {
    return ahrs.getPitch();
  }

  public double getPitch()
  {
    return ahrs.getRoll();
  }
  
  public Rotation2d getHeading()
  {
    return Rotation2d.fromDegrees(ahrs.getYaw());
  }
  public double getRoll()
  {
    return ahrs.getPitch();
  }
  public double getPitch()
  {
    return ahrs.getRoll();
  }
  public double getYaw()
  {
    return ahrs.getYaw();
  }
  /**
   * Resets the Navx.
   */
  public void reset()
  {
    ahrs.reset();
  }
  /**
   * Calibrates the Navx. Requires 10ish seconds.
   */
  public void calibrate()
  {
    ahrs.calibrate();
  }
  /**
   * Implementation of SubsystemBase.periodic(). Puts the roll, pitch, and yaw
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roll", getRoll());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Yaw", getYaw());
    // This method will be called once per scheduler run
  }
}

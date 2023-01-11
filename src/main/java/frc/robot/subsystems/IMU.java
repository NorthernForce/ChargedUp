// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMU extends SubsystemBase {
  private final AHRS ahrs = new AHRS();
  /** Creates a new IMU. */
  public IMU() {
    ahrs.reset();
  }

  public Rotation3d getRotation()
  {
    return new Rotation3d(Math.toRadians(ahrs.getRoll()), Math.toRadians(ahrs.getPitch()), Math.toRadians(ahrs.getYaw()));
  }

  public void reset()
  {
    ahrs.reset();
  }

  public void calibrate()
  {
    ahrs.calibrate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roll", ahrs.getRoll());
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    // This method will be called once per scheduler run
  }
}

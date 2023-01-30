// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMU extends SubsystemBase {
  private AHRS ahrs = new AHRS();
  /** Creates a new IMU. */
  public IMU() {
  }
  public double getPitch() {
    return ahrs.getRoll();
  }
  //Roll=Pitch for NAVx.
  public double getYaw() {
    return ahrs.getYaw();
  }
  // Yaw stays the same, don't ask me why.
  public double getRoll() {
    return ahrs.getPitch();
  }
  // Pitch = Roll on NAVx
  public AHRS getAhrs() {
    return ahrs;
  }
  
  public void setAhrs(AHRS ahrs) {
    this.ahrs = ahrs;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d();
  }
  public void calibrate() {
    ahrs.calibrate();
  }
  public boolean isCalibrating() {
    return ahrs.isCalibrating();
  }
}

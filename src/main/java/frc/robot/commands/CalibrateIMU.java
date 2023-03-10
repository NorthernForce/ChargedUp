// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.imu;
/**
 * Calibrates the IMU.
 */
public class CalibrateIMU extends CommandBase {
  /** Creates a new CalibrateIMU. */
  public CalibrateIMU() {
    addRequirements(imu);
  }
  /**
   * Initializes calibration and starts a timer.
   */
  @Override
  public void initialize() {
    imu.calibrate();
  }
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}
  /**
   * Returns true if gyro is done calibrating
   */
  @Override
  public boolean isFinished() {
    return imu.isCalibrating();
  }
}

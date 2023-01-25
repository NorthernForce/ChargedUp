// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.navigation;
/**
 * Calibrates the IMU.
 */
public class CalibrateIMU extends CommandBase {
  /** Creates a new CalibrateIMU. */
  public CalibrateIMU() {
    addRequirements(navigation);
  }
  /**
   * Initializes calibration and starts a timer.
   */
  @Override
  public void initialize() {
    navigation.calibrate();
  }
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}
  /**
   * Returns true if more than 10 seconds have passed.
   */
  @Override
  public boolean isFinished() {
    return navigation.isCalibrating();
  }
}

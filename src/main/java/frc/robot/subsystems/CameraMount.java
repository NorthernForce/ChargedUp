// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraMount extends SubsystemBase {
  private final Servo zAxisRotate = new Servo(0);
  private final Servo yAxisRotate = new Servo(1);
  /** Creates a new CameraMount. */
  public CameraMount() {
  }
  /**
   * Gets the rotate value of the z-axis servo.
   * @return z-axis rotate in degrees
   */
  public double getZAxisRotate()
  {
    return zAxisRotate.getAngle();
  }
  /**
   * Gets the rotate value of the y-axis servo.
   * @return y-axis rotate in degrees
   */
  public double getYAxisRotate()
  {
    return yAxisRotate.getAngle();
  }
  /**
   * sets camera mount z-axis angle
   * @param rotate the degree value for the new angle
   */
  public void setZAxisRotate(double rotate)
  {
    zAxisRotate.setAngle(rotate);
  }
  /**
   * sets camera mount y-axis angle
   * @param rotate the degree value for the new angle
   */
  public void setYAxisRotate(double rotate)
  {
    yAxisRotate.setAngle(rotate);
  }
  @Override
  public void periodic() {
  }
}

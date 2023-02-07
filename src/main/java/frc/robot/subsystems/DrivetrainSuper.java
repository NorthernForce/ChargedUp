// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DrivetrainSuper extends SubsystemBase
{
  /** 
   * Creates a new DrivetrainSuper. 
   * This is the parent class for other drivetrains 
   */
  public DrivetrainSuper() {}

  /**
   * Drives the robot forward applying the speed proportions
   * @param speed forward speed [1.0.. -1.0]
   * @param rotation rotational speed [1.0.. -1.0]
   */
  public void drive(double speed, double rotation) {}

  /**
   * Sets the speed proportions
   * @param speedProportion Forward speed proportion
   * @param rotationSpeedProportion Rotational speed proportion
   */
  public void setSpeedProportions(double speedProportion, double rotationSpeedProportion) {}

  /**
   * Gets the forward speed proportion
   * @return Forward speed proportion
   */
  public double getSpeedProportion()
  {
    return 0;
  }

  /**
   * Gets the Rotational Speed Proportion
   * @return the rotational speed proportion
   */
  public double getRotationSpeedProportion()
  {
    return 0;
  }

  /**
   * Drives using speeds without proportions
   * @param speed forward speed [1.0.. -1.0]
   * @param rotation rotational speed [1.0.. -1.0]
   */
  public void driveUsingSpeeds(double speed, double rotation) {}

  /**
   * Resets the encoder rotations to (0, 0)
   */
  public void resetEncoderRotations() {}

  /**
   * Gets the current encoder rotations
   * @return an array of two encoder rotations (one for each side)
   */
  public double[] getEncoderRotations()
  {
    return new double[] {0,0};
  }

  /**
   * Gets the distance traveled by the left encoder
   * @return left encoder distance in meters
   */
  public double getLeftDistance()
  {
    return 0;
  }

  /**
   * Gets the distance traveled by the right encoder
   * @return right encoder distance in meters
   */
  public double getRightDistance()
  {
    return 0;
  }

  /**
   * Gets the speed that the wheels are moving at
   * @return DifferentialDriveWheelSpeeds in m/s
   */
  public DifferentialDriveWheelSpeeds getSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(0,0);
  }

  /**
   * Drives the drivetrain based on voltage amounts
   * @param left amount of voltage to go into the left motors
   * @param right amount of voltage to go into the right motors
   */
  public void driveVolts(double left, double right) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
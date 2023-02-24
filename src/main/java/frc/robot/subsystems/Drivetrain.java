// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivetrain extends SubsystemBase
{
  /** 
   * Creates a new Drivetrain. 
   * This is the parent class for drivetrain variants 
   */
  public Drivetrain() {}
  /**
   * Drives the robot forward applying the speed proportions
   * @param speed forward speed [1.0.. -1.0]
   * @param rotation rotational speed [1.0.. -1.0]
   */
  public abstract void drive(double speed, double rotation);
  /**
   * Sets the speed proportions
   * @param speedProportion Forward speed proportion
   * @param rotationSpeedProportion Rotational speed proportion
   */
  public abstract void setSpeedProportions(double speedProportion, double rotationSpeedProportion);
  /**
   * Gets the forward speed proportion
   * @return Forward speed proportion
   */
  public abstract double getSpeedProportion();
  /**
   * Gets the Rotational Speed Proportion
   * @return the rotational speed proportion
   */
  public abstract double getRotationSpeedProportion();
  /**
   * Drives using speeds without proportions
   * @param speed forward speed [1.0.. -1.0]
   * @param rotation rotational speed [1.0.. -1.0]
   */
  public abstract void driveUsingSpeeds(double speed, double rotation);
  /**
   * Resets the encoder rotations to (0, 0)
   */
  public abstract void resetEncoderRotations();
  /**
   * Gets the current encoder rotations
   * @return an array of two encoder rotations (one for each side)
   */
  public abstract double[] getEncoderRotations();
  /**
   * Gets the distance traveled by the left encoder
   * @return left encoder distance in meters
   */
  public abstract double getLeftDistance();
  /**
   * Gets the distance traveled by the right encoder
   * @return right encoder distance in meters
   */
  public abstract double getRightDistance();
  /**
   * Gets the speed that the wheels are moving at
   * @return DifferentialDriveWheelSpeeds in m/s
   */
  public abstract DifferentialDriveWheelSpeeds getSpeeds();
  /**
   * Drives the drivetrain based on voltage amounts
   * @param left amount of voltage to go into the left motors
   * @param right amount of voltage to go into the right motors
   */
  public abstract void driveVolts(double left, double right);

  public abstract void driveUsingChassisSpeeds(ChassisSpeeds speeds);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
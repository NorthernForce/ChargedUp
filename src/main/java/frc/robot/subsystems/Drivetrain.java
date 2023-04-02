// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Motors.MotorGroup;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Drivetrain extends SubsystemBase
{
  private MotorGroup leftSide;
  private MotorGroup rightSide;
  private DifferentialDrive robotDrive;
  private final DifferentialDriveKinematics kinematics;
  private final SimpleMotorFeedforward feedforward;
  private double speedProportion = 1.0, rotationSpeedProportion = 0.75;
  /** 
   * Creates a new Drivetrain. 
   */
  public Drivetrain(MotorGroup leftSide, MotorGroup rightSide, double trackWidth, double kS, double kV, double kA) {
    this.leftSide = leftSide;
    this.rightSide = rightSide;
    robotDrive = new DifferentialDrive(leftSide, rightSide);
    kinematics = new DifferentialDriveKinematics(trackWidth);
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    Shuffleboard.getTab("Drivetrain").addNumber("Speed (ftps)", () -> Units.metersToFeet(
      (getSpeeds().leftMetersPerSecond + getSpeeds().rightMetersPerSecond) / 2
    )).withPosition(0, 0);

  }
  /**
   * Drives the robot forward applying the speed proportions
   * @param speed forward speed [1.0.. -1.0]
   * @param rotation rotational speed [1.0.. -1.0]
   */
  public void drive(double speed, double rotation) {
    robotDrive.arcadeDrive(speed * speedProportion, rotation * rotationSpeedProportion);
  }
  /**
   * Sets the speed proportions
   * @param speedProportion Forward speed proportion
   * @param rotationSpeedProportion Rotational speed proportion
   */
  public void setSpeedProportions(double speedProportion, double rotationSpeedProportion) {
    this.speedProportion = speedProportion;
    this.rotationSpeedProportion = rotationSpeedProportion;
  }
  /**
   * Gets the forward speed proportion
   * @return Forward speed proportion
   */
  public double getSpeedProportion()
  {
    return speedProportion;
  }
  /**
   * Gets the Rotational Speed Proportion
   * @return the rotational speed proportion
   */
  public double getRotationSpeedProportion()
  {
    return rotationSpeedProportion;
  }
  /**
   * Drives using speeds without proportions
   * @param speed forward speed [1.0.. -1.0]
   * @param rotation rotational speed [1.0.. -1.0]
   */
  public void driveUsingSpeeds(double speed, double rotation)
  {
    robotDrive.arcadeDrive(speed, rotation);
  }
  /**
   * Resets the encoder rotations to (0, 0)
   */
  public void resetEncoderRotations() {
    leftSide.resetEncoderRotations();
    rightSide.resetEncoderRotations();
  }
  /**
    {
      leftSide.getEncoderRotations(), 
      rightSide.getEncoderRotations()
    };
  }
  /**
   * Gets the distance traveled by the left encoder
   * @return left encoder distance in meters
   */
  public double getLeftDistance()
  {
    return leftSide.getEncoderRotations() * Constants.DrivetrainConstants.METERS_PER_REVOLUTION;
  }
  /**
   * Gets the distance traveled by the right encoder
   * @return right encoder distance in meters
   */
  public double getRightDistance()
  {
    return rightSide.getEncoderRotations() * Constants.DrivetrainConstants.METERS_PER_REVOLUTION;
  }
  /**
   * Gets the speed that the wheels are moving at
   * @return DifferentialDriveWheelSpeeds in m/s
   */
  public DifferentialDriveWheelSpeeds getSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(
      leftSide.getEncoderRPS() * Constants.DrivetrainConstants.METERS_PER_REVOLUTION,
      rightSide.getEncoderRPS() * Constants.DrivetrainConstants.METERS_PER_REVOLUTION
    );
  }
  /**
   * Drives the drivetrain based on voltage amounts
   * @param left amount of voltage to go into the left motors
   * @param right amount of voltage to go into the right motors
   */
  public void driveVolts(double left, double right) {
    leftSide.setVoltage(left);
    rightSide.setVoltage(right);
    robotDrive.feed();
  }
  /**
   * Drives using meters per second
   * @param speeds
   */
  public void driveUsingChassisSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds driveSpeeds = kinematics.toWheelSpeeds(speeds);
    leftSide.setVoltage(feedforward.calculate(driveSpeeds.leftMetersPerSecond));
    rightSide.setVoltage(feedforward.calculate(driveSpeeds.rightMetersPerSecond));
    robotDrive.feed();
  }
  /** 
   * Returns SimpleMotorFeedforward
   * @return feedforward
   */
  public SimpleMotorFeedforward getFeedforward()
  {
    return feedforward;
  }
  /** 
   * Returns DifferentialDriveKinematics
   * @return kinematics
   */
  public DifferentialDriveKinematics getKinematics()
  {
    return kinematics;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
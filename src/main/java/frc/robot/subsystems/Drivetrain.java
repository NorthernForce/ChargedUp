// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftPrimary;
  private final CANSparkMax rightPrimary;
  private final CANSparkMax leftFollower;
  private final CANSparkMax rightFollower;
  private final DifferentialDrive robotDrive;
  private double speedProportion = 1.0, rotationSpeedProportion = 0.75;
  /**
   * Constructs a drivetrain
   */
  public Drivetrain() {
    leftPrimary = new CANSparkMax(1, MotorType.kBrushless);
    rightPrimary = new CANSparkMax(2, MotorType.kBrushless);
    leftFollower = new CANSparkMax(3, MotorType.kBrushless);
    rightFollower = new CANSparkMax(4, MotorType.kBrushless);
    setFollowers();
    setInvert();
    configureAllControllers();
    robotDrive = new DifferentialDrive(leftPrimary, rightPrimary);
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
  public void setSpeedProportions(double speedProportion, double rotationSpeedProportion)
  {
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
  public void driveUsingSpeeds(double speed, double rotation) {
    robotDrive.arcadeDrive(speed, rotation);
  }
  /**
   * Resets the encoder rotations to (0, 0)
   */
  public void resetEncoderRotations()
  {
    leftPrimary.getEncoder().setPosition(0);
    rightPrimary.getEncoder().setPosition(0);
  }
  /**
   * Gets the current encoder rotations
   * @return an array of two encoder rotations (one for each side)
   */
  public double[] getEncoderRotations() {
    double leftSideRotations = (leftPrimary.getEncoder().getPosition() * -1);
    double rightSideRotations = rightPrimary.getEncoder().getPosition();
    return new double[] {leftSideRotations, rightSideRotations};
  }
  /**
   * Gets the distance traveled by the left encoder
   * @return left encoder distance in meters
   */
  public double getLeftDistance()
  {
    return (-leftPrimary.getEncoder().getPosition()) * METERS_PER_REVOLUTION;
  }
  /**
   * Gets the distance traveled by the right encoder
   * @return right encoder distance in meters
   */
  public double getRightDistance()
  {
    return (rightPrimary.getEncoder().getPosition() / 2048) * METERS_PER_REVOLUTION;
  }
  /**
   * Sets the two secondary motors to follow the primary motors
   */
  private void setFollowers() {
    leftFollower.follow(leftPrimary);
    rightFollower.follow(rightPrimary);
  }
  /**
   * Sets the right side as inverted whereas the left is not
   */
  private void setInvert() {
    rightPrimary.setInverted(false);
    leftPrimary.setInverted(true);
  }
  /**
   * Configures each of the PID Controllers for the motors
   */
  private void configureAllControllers() {
    configureController(leftPrimary, false);
    configureController(rightPrimary, false);
    configureController(leftFollower, true);
    configureController(rightFollower, true);
  }
  /**
   * Factory resets each controller
   * @param controller the controller to reset
   * @param isFollower whether it is a follower
   */
  private void configureController(CANSparkMax controller, Boolean isFollower) {
    controller.setIdleMode(IdleMode.kBrake);
  }
  /**
   * Gets the speed that the wheels are moving at
   * @return DifferentialDriveWheelSpeeds in m/s
   */
  public DifferentialDriveWheelSpeeds getSpeeds()
  {
    double leftVelocity = ((-leftPrimary.getEncoder().getVelocity())) * 60 * METERS_PER_REVOLUTION;
    double rightVelocity = ((rightPrimary.getEncoder().getVelocity()) / 2048) * 60 * METERS_PER_REVOLUTION;
    return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
  }
  /**
   * Drives the drivetrain based on voltage amounts
   * @param left amount of voltage to go into the left motors
   * @param right amount of voltage to go into the right motors
   */
  public void driveVolts(double left, double right)
  {
    leftPrimary.setVoltage(left);
    rightPrimary.setVoltage(right);
    robotDrive.feed();
  }
  /**
   * Implementation of SubsystemBase.periodic(). Does nothing.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
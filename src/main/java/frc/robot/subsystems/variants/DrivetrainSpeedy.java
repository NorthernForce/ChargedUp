// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.subsystems.variants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class DrivetrainSpeedy extends Drivetrain {
  private final WPI_TalonFX leftPrimary;
  private final WPI_TalonFX rightPrimary;
  private final WPI_TalonFX leftFollower;
  private final WPI_TalonFX rightFollower;
  private final DifferentialDrive robotDrive;
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  private double speedProportion = 1.0, rotationSpeedProportion = 0.75;
  /**
   * Constructs a drivetrain
   */
  public DrivetrainSpeedy() {
    leftPrimary = new WPI_TalonFX(LEFT_PRIMARY_ID);
    rightPrimary = new WPI_TalonFX(RIGHT_PRIMARY_ID);
    leftFollower = new WPI_TalonFX(LEFT_FOLLOWER_ID);
    rightFollower = new WPI_TalonFX(RIGHT_FOLLOWER_ID);
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
    leftPrimary.getSensorCollection().setIntegratedSensorPosition(0, 0);
    rightPrimary.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }
  /**
   * Gets the current encoder rotations
   * @return an array of two encoder rotations (one for each side)
   */
  public double[] getEncoderRotations() {
    double leftSideRotations = (leftPrimary.getSensorCollection().getIntegratedSensorPosition() * -1) / 2048;
    double rightSideRotations = rightPrimary.getSensorCollection().getIntegratedSensorPosition() / 2048;
    return new double[] {leftSideRotations, rightSideRotations};
  }
  /**
   * Gets the distance traveled by the left encoder
   * @return left encoder distance in meters
   */
  public double getLeftDistance()
  {
    return (-leftPrimary.getSensorCollection().getIntegratedSensorPosition() / 2048) * METERS_PER_REVOLUTION;
  }
  /**
   * Gets the distance traveled by the right encoder
   * @return right encoder distance in meters
   */
  public double getRightDistance()
  {
    return (rightPrimary.getSensorCollection().getIntegratedSensorPosition() / 2048) * METERS_PER_REVOLUTION;
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
    rightPrimary.setInverted(true);
    rightFollower.setInverted(InvertType.FollowMaster);
    leftPrimary.setInverted(false);
    leftFollower.setInverted(InvertType.FollowMaster);
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
  private void configureController(WPI_TalonFX controller, Boolean isFollower) {
    final double currentLimit = 60;
    final double limitThreshold = 90;
    final double triggerThreshTimeInSec = 1;
    controller.configFactoryDefault();
    controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, limitThreshold, triggerThreshTimeInSec));
    if (!isFollower) {
      controller.configClosedloopRamp(DRIVE_RAMP_RATE);
      controller.configOpenloopRamp(DRIVE_RAMP_RATE);
    }
    controller.setNeutralMode(NeutralMode.Brake);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    controller.configAllSettings(configs);
  }
  /**
   * Gets the speed that the wheels are moving at
   * @return DifferentialDriveWheelSpeeds in m/s
   */
  public DifferentialDriveWheelSpeeds getSpeeds()
  {
    double leftVelocity = ((-leftPrimary.getSelectedSensorVelocity()) / 2048) * 10 * METERS_PER_REVOLUTION;
    double rightVelocity = ((rightPrimary.getSelectedSensorVelocity()) / 2048) * 10 * METERS_PER_REVOLUTION;
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
  @Override
  public void driveUsingChassisSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds driveSpeeds = kinematics.toWheelSpeeds(speeds);
    leftPrimary.setVoltage(feedforward.calculate(driveSpeeds.leftMetersPerSecond));
    rightPrimary.setVoltage(feedforward.calculate(driveSpeeds.rightMetersPerSecond));
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
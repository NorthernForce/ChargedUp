// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Arm extends SubsystemBase {
  // We know we will have two talons
  private final WPI_TalonFX leftMotor, rightMotor;
  private final AnalogPotentiometer potentiometer;
  /**
   * Configures a controller
   * @param controller motor controller
   * @param isFollower is a follower
   */
  private void configureController(WPI_TalonFX controller, Boolean isFollower) {
    final double currentLimit = 60;
    final double limitThreshold = 90;
    final double triggerThreshTimeInSec = 1;
    controller.configFactoryDefault();
    controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
      currentLimit, limitThreshold, triggerThreshTimeInSec));
    if (!isFollower) {
      controller.configClosedloopRamp(Constants.ARM_RAMP_RATE);
      controller.configOpenloopRamp(Constants.ARM_RAMP_RATE);
    }
    controller.setNeutralMode(NeutralMode.Brake);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    controller.configAllSettings(configs);
  }
  /** Creates a new Arm. */
  public Arm() {
    // Assert required subsystem have been declared
    assert pcm != null;
    leftMotor = new WPI_TalonFX(Constants.ARM_LEFT_MOTOR);
    rightMotor = new WPI_TalonFX(Constants.ARM_RIGHT_MOTOR);
    potentiometer = new AnalogPotentiometer(Constants.ARM_POTENTIOMETER_ID, 180, -90);
    leftMotor.setInverted(TalonFXInvertType.OpposeMaster);
    leftMotor.follow(rightMotor);
    configureController(leftMotor, true);
    configureController(rightMotor, false);
  }
  /** Extends Arm */
  public void extend()
  {
  }
  /** Retracts Arm */
  public void retract()
  {
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(potentiometer.get());
  }
  /**
   * Set arm angle
   * @param angle angle to set the arm to
  */
  public void setAngle(Rotation2d angle)
  {
  }
  /**
   * Tells whether the arm is extended
   * @return whether the arm is extended
   */
  public boolean isExtended()
  {
    return false;
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
  }
  /**
   * Gets the current translation of the end of the arm from the robot orign
   * @return Translation3d
   */
  public Translation3d getArmTranslation()
  {
    return Constants.ARM_ORIGIN.plus(new Translation3d(
      isExtended() ? Constants.ARM_EXTENDED_LENGTH : Constants.ARM_RETRACTED_LENGTH,
      new Rotation3d(0, getAngle().getRadians(), 0)
    ));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", arm.getAngle().getDegrees());
  }
}

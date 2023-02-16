// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private final WPI_TalonFX extensionMotor1;
  private final WPI_TalonFX extensionMotor2;
  private final ArmFeedforward armFeedforward;
  private final PIDController rotateController, extensionController1, extensionController2;
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
    leftMotor.setInverted(TalonFXInvertType.OpposeMaster);
    leftMotor.follow(rightMotor);
    configureController(leftMotor, true);
    configureController(rightMotor, false);
    armFeedforward = new ArmFeedforward(Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV,
      Constants.ARM_KG);
    rotateController = new PIDController(Constants.ARM_PROPORTION, 0, 0);
    rotateController.setSetpoint(Constants.ARM_STARTING_ROTATION.getRadians());
    extensionMotor1 = new WPI_TalonFX(Constants.ARM_EXTENSION_MOTOR_1_ID);
    extensionMotor2 = new WPI_TalonFX(Constants.ARM_EXTENSION_MOTOR_2_ID);
    configureController(extensionMotor1, false);
    configureController(extensionMotor2, false);
    extensionController1 = new PIDController(Constants.ARM_EXTENSION_PROPORTION, 0, 0);
    extensionController2 = new PIDController(Constants.ARM_EXTENSION_PROPORTION, 0, 0);
  }
  /** Extends Arm */
  public void extend()
  {
    extensionController1.setSetpoint(Constants.ARM_INNER_EXTENDED_LENGTH);
    extensionController2.setSetpoint(Constants.ARM_OUTER_EXTENDED_LENGTH);
  }
  /** Retracts Arm */
  public void retract()
  {
    extensionController1.setSetpoint(Constants.ARM_INNER_RETRACTED_LENGTH);
    extensionController2.setSetpoint(Constants.ARM_OUTER_RETRACTED_LENGTH);
  }
  public void setArmExtension(double distance)
  {
    extensionController1.setSetpoint(distance / 2);
    extensionController2.setSetpoint(distance / 2);
  }
  /**
   * Gets the total length of the extended arm
   * @return total extended arm length
   */
  public double getExtendedArmLength()
  {
    return getInnerExtendedArmLength() + getOuterExtendedArmLength();
  }
  private double getInnerExtendedArmLength()
  {
    return 0;
  }
  private double getOuterExtendedArmLength()
  {
    return 0;
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return null;
  }
  /**
   * Set arm angle
   * @param angle angle to set the arm to
  */
  public void setAngle(Rotation2d angle)
  {
    rotateController.setSetpoint(angle.getRadians());
  }
  /**
   * Tells whether the arm is extended
   * @return whether the arm is extended
   */
  public boolean isExtended()
  {
    return Math.abs(getInnerExtendedArmLength() - Constants.ARM_INNER_EXTENDED_LENGTH) >= 0.05
      && Math.abs(getOuterExtendedArmLength() - Constants.ARM_OUTER_EXTENDED_LENGTH) >= 0.05;
  }
  /**
   * Tells whether the arm is retracted
   * @return whether the arm is retracted
   */
  public boolean isRetracted()
  {
    return Math.abs(getInnerExtendedArmLength() - Constants.ARM_INNER_RETRACTED_LENGTH) >= 0.05
      && Math.abs(getOuterExtendedArmLength() - Constants.ARM_OUTER_RETRACTED_LENGTH) >= 0.05;
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
    rotateController.setSetpoint(rotateController.getSetpoint() + Rotation2d.fromDegrees(speed).getRadians());
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
    rightMotor.setVoltage(armFeedforward.calculate(getAngle().getRadians(), 0)
      + rotateController.calculate(getAngle().getRadians()));
    SmartDashboard.putNumber("Arm Angle", arm.getAngle().getDegrees());
  }
}

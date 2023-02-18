// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
  // We know we will have two talons
  private final WPI_TalonFX rotateMotor;
  private final CANSparkMax extensionMotor;
  private final PIDController rotateController, extensionController;
  /** Creates a new Arm. */
  public Arm() {
    // Assert required subsystem have been declared
    rotateMotor = new WPI_TalonFX(Constants.ARM_LEFT_MOTOR);
    rotateMotor.setInverted(TalonFXInvertType.CounterClockwise);
    configureController(rotateMotor, false);
    rotateMotor.setSelectedSensorPosition(Constants.ARM_STARTING_ROTATION.getRotations() * Constants.ROTATE_GEAR_RATIO * 2048);
    rotateController = new PIDController(Constants.ARM_PROPORTION, 0, 0);
    rotateController.setSetpoint(Constants.ARM_STARTING_ROTATION.getRadians());
    extensionMotor = new CANSparkMax(Constants.ARM_EXTENSION_MOTOR_ID, MotorType.kBrushless);
    extensionMotor.setInverted(false);
    extensionController = new PIDController(Constants.ARM_EXTENSION_PROPORTION, 0, 0);
    extensionMotor.getEncoder().setPositionConversionFactor(Constants.EXTENSION_DISTANCE_PER_ROTATION / Constants.EXTENSION_GEAR_RATIO);
    extensionMotor.getEncoder().setPosition(Constants.EXTENSION_STARTING_DISTANCE);
  }
  /** Extends Arm */
  public void extend()
  {
    extensionController.setSetpoint(Constants.ARM_EXTENDED_LENGTH);
  }
  /** Retracts Arm */
  public void retract()
  {
    extensionController.setSetpoint(Constants.ARM_RETRACTED_LENGTH);
  }
  /**
   * Set length to extend to
   * @param distance distance meters
   */
  public void setArmExtension(double distance)
  {
    extensionController.setSetpoint(distance);
  }
  /**
   * Gets the total length of the extended arm
   * @return total extended arm length
   */
  public double getExtendedArmLength()
  {
    return extensionMotor.getEncoder().getPosition();
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromRotations((rotateMotor.getSelectedSensorPosition() / 2048)
      / Constants.ROTATE_GEAR_RATIO);
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
    return Math.abs(getExtendedArmLength() - Constants.ARM_EXTENDED_LENGTH) >= 0.05;
  }
  /**
   * Tells whether the arm is retracted
   * @return whether the arm is retracted
   */
  public boolean isRetracted()
  {
    return Math.abs(getExtendedArmLength() - Constants.ARM_RETRACTED_LENGTH) >= 0.05;
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
      getExtendedArmLength(),
      new Rotation3d(0, getAngle().getRadians(), 0)
    ));
  }
  @Override
  public void periodic() {
    rotateMotor.set(rotateController.calculate(getAngle().getRadians()));
    extensionMotor.set(extensionController.calculate(getExtendedArmLength()));
    SmartDashboard.putNumber("Arm Angle", arm.getAngle().getDegrees());
  }
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
}

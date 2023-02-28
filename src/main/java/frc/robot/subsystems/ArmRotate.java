// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class ArmRotate extends SubsystemBase {
  // We know we will have two talons
  private final WPI_TalonFX rotateMotor;
  private final PIDController rotateController;
  private final CANCoder rotateEncoder;
  /** Creates a new Arm. */
  public ArmRotate() {
    // Assert required subsystem have been declared

    rotateMotor = new WPI_TalonFX(Constants.ARM_MOTOR_ID);
    rotateMotor.setInverted(TalonFXInvertType.CounterClockwise);
    configureController(rotateMotor, false);
    rotateController = new PIDController(Constants.ARM_PROPORTION, 0, 0);

    rotateEncoder = new CANCoder(Constants.ARM_ROTATE_CANCODER_ID);
    Shuffleboard.getTab("Arm").addDouble("Angle", () -> getAngle().getDegrees());
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(rotateEncoder.getPosition());
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
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
    rotateController.setSetpoint(rotateController.getSetpoint() + Rotation2d.fromDegrees(speed).getRadians());
  }
  @Override
  public void periodic() {
    rotateMotor.set(rotateController.calculate(getAngle().getRadians()));
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
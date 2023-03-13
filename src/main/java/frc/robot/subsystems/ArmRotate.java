// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Motors.MotorGroupTalonFX;
import frc.robot.Constants;


import frc.lib.Motors.MotorGroupTalonFX;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class ArmRotate extends SubsystemBase {
  // We know we will have two talons
  private final MotorGroupTalonFX talonGroup;
  private final CANCoder rotateEncoder;
  private final GenericEntry kFEntry, kPEntry, kIEntry, kDEntry;
  /** Creates a new Arm. */
  public ArmRotate() {
    talonGroup = new MotorGroupTalonFX(Constants.ArmConstants.PRIMARY_MOTOR_ID, Constants.ArmConstants.FOLLOWER_MOTOR_ID);
    talonGroup.setFollowerOppose(0);
    talonGroup.configClosedLoop(
      0, 0,
      Constants.ArmConstants.kF, Constants.ArmConstants.kP,
      Constants.ArmConstants.kI, Constants.ArmConstants.kD
    );
    talonGroup.configSelectedProfile(0, 0);
    rotateEncoder = new CANCoder(Constants.ArmConstants.CANCODER_ID);
    rotateEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rotateEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    talonGroup.linkAndUseCANCoder(rotateEncoder);
    talonGroup.setLimits(Constants.ArmConstants.BACKWARD_LIMIT, Constants.ArmConstants.FORWARD_LIMIT);
    Shuffleboard.getTab("Arm").addDouble("Angle", () -> getAngle().getDegrees()).withPosition(0, 0);
    kFEntry = Shuffleboard.getTab("Arm").add("kF", Constants.ArmConstants.kF).getEntry();
    kPEntry = Shuffleboard.getTab("Arm").add("kP", Constants.ArmConstants.kP).getEntry();
    kIEntry = Shuffleboard.getTab("Arm").add("kI", Constants.ArmConstants.kI).getEntry();
    kDEntry = Shuffleboard.getTab("Arm").add("kD", Constants.ArmConstants.kD).getEntry();
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromRotations(talonGroup.getEncoderRotations() + Constants.ArmConstants.CANCODER_OFFSET);
  }
  /**
   * @param angle angle to set the arm to
  */
  public void setAngle(Rotation2d angle) {
    setAngle(angle, false);
  }
  /**
   * Set arm angle. Should be called repeatedly.
   * @param angle angle to set the arm to
   * @param ignoreLimit Whether or not the set angle can ignore angle limits
  */
  public void setAngle(Rotation2d angle, boolean ignoreLimit)
  {
    talonGroup.setPosition(angle.getRotations() - Constants.ArmConstants.CANCODER_OFFSET, getAngle().getCos() * Constants.ArmConstants.kFF);
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
    talonGroup.setPercent(speed, getAngle().getCos() * Constants.ArmConstants.kFF);
  }
  /**
    * Calibrates the arm magnetic offset to a known point.
    * @param position The known point to calibrate to.
   */
  public void calibrate(Rotation2d position)
  {
    rotateEncoder.configMagnetOffset(-rotateEncoder.getAbsolutePosition());
  }
  @Override
  public void periodic() {
    talonGroup.configClosedLoop(
      0,
      0,
      kFEntry.getDouble(Constants.ArmConstants.kF),
      kPEntry.getDouble(Constants.ArmConstants.kP),
      kIEntry.getDouble(Constants.ArmConstants.kI),
      kDEntry.getDouble(Constants.ArmConstants.kD)
    );
  }
} 
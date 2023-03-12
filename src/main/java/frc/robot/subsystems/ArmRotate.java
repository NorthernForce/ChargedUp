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

import com.ctre.phoenix.sensors.CANCoder;

public class ArmRotate extends SubsystemBase {
  // We know we will have two talons
  private final MotorGroupTalonFX talonGroup;
  private final CANCoder rotateEncoder;
  private final GenericEntry kFEntry, kPEntry, kIEntry, kDEntry;
  /** Creates a new Arm. */
  public ArmRotate() {
    talonGroup = new MotorGroupTalonFX(Constants.ARM_PRIMARY_MOTOR_ID, new int[]
    {
      Constants.ARM_FOLLOWER_MOTOR_ID
    });
    talonGroup.setFollowerOppose(0);
    talonGroup.configClosedLoop(0, 0, Constants.ARM_KF, Constants.ARM_KP, Constants.ARM_KI, Constants.ARM_KD);
    talonGroup.configSelectedProfile(0, 0);
    rotateEncoder = new CANCoder(Constants.ARM_ROTATE_CANCODER_ID);
    talonGroup.linkAndUseCANCoder(rotateEncoder);
    talonGroup.setLimits(Constants.ARM_BACKWARD_LIMIT, Constants.ARM_FOWARD_LIMIT);
    Shuffleboard.getTab("Arm").addDouble("Angle", () -> getAngle().getDegrees()).withPosition(0, 0);
    kFEntry = Shuffleboard.getTab("Arm").add("kF", Constants.ARM_KF).getEntry();
    kPEntry = Shuffleboard.getTab("Arm").add("kP", Constants.ARM_KP).getEntry();
    kIEntry = Shuffleboard.getTab("Arm").add("kI", Constants.ARM_KI).getEntry();
    kDEntry = Shuffleboard.getTab("Arm").add("kD", Constants.ARM_KD).getEntry();
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromRotations(talonGroup.getEncoderRotations() + Constants.CANCODER_OFFSET);
  }
  /**
   * @param angle angle to set the arm to
  */
  public void setAngle(Rotation2d angle) {
    talonGroup.setPosition(angle.getRotations() - Constants.CANCODER_OFFSET, getAngle().getCos() * Constants.ARM_KFF);
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
    talonGroup.setPercent(speed, getAngle().getCos() * Constants.ARM_KFF);
  }
  @Override
  public void periodic() {
    talonGroup.configClosedLoop(
      0,
      0,
      kFEntry.getDouble(Constants.ARM_KF),
      kPEntry.getDouble(Constants.ARM_KP),
      kIEntry.getDouble(Constants.ARM_KI),
      kDEntry.getDouble(Constants.ARM_KD)
    );
  }
} 
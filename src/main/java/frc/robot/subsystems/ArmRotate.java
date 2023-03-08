// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Motors.MotorGroupTalon;
import frc.robot.Constants;

import frc.robot.subsystems.variants.MotorGroupTalonFX;

import com.ctre.phoenix.sensors.CANCoder;

public class ArmRotate extends SubsystemBase {
  // We know we will have two talons
  private final MotorGroupTalonFX talonGroup;
  private final PIDController rotateController;
  private final CANCoder rotateEncoder;
  /** Creates a new Arm. */
  public ArmRotate() {
    talonGroup = new MotorGroupTalonFX(Constants.ARM_PRIMARY_MOTOR_ID, new int[]
    {
      Constants.ARM_FOLLOWER_MOTOR_ID
    });
    talonGroup.setFollowerOppose(0);
    rotateController = new PIDController(Constants.ARM_PROPORTION, 0, 0);
    rotateEncoder = new CANCoder(Constants.ARM_ROTATE_CANCODER_ID);
    Shuffleboard.getTab("Arm").addDouble("Angle", () -> getAngle().getDegrees()).withPosition(0, 0);
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
    talonGroup.set(rotateController.calculate(getAngle().getRadians()));
  }
}
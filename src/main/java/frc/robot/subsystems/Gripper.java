// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private final CANSparkMax motor;
  /** Creates a new Gripper. */
  public Gripper() {
    motor = new CANSparkMax(Constants.GripperConstants.MOTOR_ID, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
  }
  public void setSpeed(double speed)
  {
    motor.set(speed);
  }
  public boolean hasObject()
  {
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

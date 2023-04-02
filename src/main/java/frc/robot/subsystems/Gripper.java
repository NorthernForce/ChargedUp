// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.NameNotFoundException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Motors.MotorGroupSpark;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private CANSparkMax motor;

  /** Creates a new Gripper. */
  public Gripper() {
    try {
      motor = new MotorGroupSpark(MotorType.kBrushless, Constants.GripperConstants.MOTOR_ID);
    }
    catch (NameNotFoundException e) {
      e.printStackTrace();
      motor = null;
    }

    if (motor != null) {
      motor.setIdleMode(IdleMode.kBrake);
    }
  }
  
  public void setSpeed(double speed)
  {
    if (motor == null)
    {
      return;
    }
    
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

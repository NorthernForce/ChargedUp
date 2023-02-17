// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Wrist extends PIDSubsystem {
  private final CANSparkMax wristMotor;
  /** Creates a new Wrist. */
  public Wrist() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.WRIST_PROPORTION, 0, 0));
    wristMotor = new CANSparkMax(Constants.WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.getEncoder().setPositionConversionFactor(1 / Constants.GEAR_RATIO);
    wristMotor.getEncoder().setPosition(Rotation2d.fromDegrees(-90).getRotations());
    getController().setSetpoint(getAngle().getDegrees());
  }

  public void setAngle(Rotation2d angle)
  {
    getController().setSetpoint(angle.getDegrees());
  }

  public Rotation2d getAngle()
  {
    return Rotation2d.fromRotations(wristMotor.getEncoder().getPosition());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    wristMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return wristMotor.getEncoder().getPosition();
  }
}

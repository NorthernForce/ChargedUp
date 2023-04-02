// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.lib.Motors.MotorGroupSpark;
import javax.naming.NameNotFoundException;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase {
  private MotorGroupSpark wristMotor;
  private final GenericEntry kFEntry, kPEntry, kIEntry, kDEntry;
  private final GenericEntry kMaxAccelEntry, kMaxVelocityEntry, kMinOutputVelocityEntry;
  /** Creates a new Wrist. */
  public Wrist() {
    try {
      wristMotor = new MotorGroupSpark(MotorType.kBrushed, Constants.WristConstants.MOTOR_ID);
    } catch (NameNotFoundException e) {
      e.printStackTrace();
      wristMotor = null;
    }

    if (wristMotor != null) {
    wristMotor.setInverted(true);
    wristMotor.setLimits(WristConstants.FORWARD_LIMIT, WristConstants.BACKWARD_LIMIT);
    wristMotor.configurePID(0, Constants.WristConstants.kP, Constants.WristConstants.kI, Constants.WristConstants.kD, 5, Constants.WristConstants.kMaxAccel, Constants.WristConstants.kMaxVelocity, Constants.WristConstants.kMinOutputVelocity);
    wristMotor.setFeedbackSensor(wristMotor.getAbsoluteEncoder());
    wristMotor.getAbsoluteEncoder().setPositionConversionFactor(360);
    }
    Shuffleboard.getTab("Arm").addNumber("Wrist", () -> getAngle().getDegrees());
    kFEntry = Shuffleboard.getTab("Arm").add("Wrist - kF", Constants.WristConstants.kF).getEntry();
    kPEntry = Shuffleboard.getTab("Arm").add("Wrist - kP", Constants.WristConstants.kP).getEntry();
    kIEntry = Shuffleboard.getTab("Arm").add("Wrist - kI", Constants.WristConstants.kI).getEntry();
    kDEntry = Shuffleboard.getTab("Arm").add("Wrist - kD", Constants.WristConstants.kD).getEntry();
    kMaxAccelEntry = Shuffleboard.getTab("Arm").add("Wrist - MaxAccel", Constants.WristConstants.kMaxAccel).getEntry();
    kMaxVelocityEntry = Shuffleboard.getTab("Arm").add("Wrist - MaxVelocity", Constants.WristConstants.kMaxVelocity).getEntry();
    kMinOutputVelocityEntry = Shuffleboard.getTab("Arm").add("Wrist - MinOutputVelocity", Constants.WristConstants.kMinOutputVelocity).getEntry();
  }

  /**
   * Returns angle off of the line of the arm
   * @return Rotation2d
   */
  public Rotation2d getAngle()
  {
    if (wristMotor == null) {
      return new Rotation2d();
    }

    return Rotation2d.fromDegrees(wristMotor.getAbsolute() - 90);
  }

  public boolean isCANCoderPresent()
  {
    return true;
  }
  
  /**
   * Sets the position of the wrist motor
   * @param position Rotation to be
   */
  public void setRotation(Rotation2d rotation)
  {
    if (wristMotor == null) {
      return;
    }

    wristMotor.setUsingPID(rotation.getDegrees() + 90, 0);
  }
  
  /**
   * Sets the percent without calculating feedforward
   * @param percent [-1.0.. 1.0]
   */
  public void setPercent(double percent)
  {
    if (wristMotor == null) {
      return;
    }

    wristMotor.set(percent);
  }
  public Rotation2d getVelocity()
  {
    if (wristMotor == null) {
      return new Rotation2d();
    }
    
    return Rotation2d.fromRotations(wristMotor.getAbsoluteRPS());
  }
  /**
   * Calibrates the wrist CANCoder with a known angle
   * @param angle the known angle
   */
  public void calibrate(Rotation2d angle)
  {
    if (wristMotor == null)
    {
      return;
    }
    wristMotor.resetAbsolute(angle.getRotations());
  }
  @Override
  public void periodic()
  {
    if (wristMotor == null)
    {
      return;
    }
    wristMotor.configurePID(
      0,  
      kPEntry.getDouble(Constants.WristConstants.kP),
      kIEntry.getDouble(Constants.WristConstants.kI),
      kDEntry.getDouble(Constants.WristConstants.kD),
      0,
      kMaxAccelEntry.getDouble(Constants.WristConstants.kMaxAccel),
      kMaxVelocityEntry.getDouble(Constants.WristConstants.kMaxVelocity),
      kMinOutputVelocityEntry.getDouble(Constants.WristConstants.kMinOutputVelocity)
    );
  }
}

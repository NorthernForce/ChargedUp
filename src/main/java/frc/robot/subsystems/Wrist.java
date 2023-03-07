// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.variants.MotorGroupTalonSRX;
import static frc.robot.RobotContainer.*;

public class Wrist extends SubsystemBase {
  private final MotorGroupTalonSRX srx = new MotorGroupTalonSRX(Constants.WRIST_MOTOR_ID);
  /** Creates a new Wrist. */
  public Wrist() {
    srx.configClosedLoop(0, 0, Constants.WRIST_KF, Constants.WRIST_KP, Constants.WRIST_KI, Constants.WRIST_KD);
    srx.configSelectedSlot(0, 0);
    Shuffleboard.getTab("Arm").addNumber("Wrist", () -> getAngle().getDegrees());
  }
  /**
   * Returns angle off of the line of the arm
   * @return Rotation2d
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(srx.getEncoderRotations() / Constants.WRIST_GEAT_RATIO);
  }
  /**
   * Sets the velocity of the wrist motor
   * @param velocity degrees/sec
   */
  public void setVelocity(double speed)
  {
    srx.setVelocity(speed, armRotate.getAngle().plus(getAngle()).getCos() * Constants.WRIST_KFF);
  }
  /**
   * Sets the position of the wrist motor
   * @param position Rotation to be
   */
  public void setRotation(Rotation2d rotation)
  {
    srx.setPosition(rotation.getRotations() / Constants.GEAR_RATIO, armRotate.getAngle().plus(getAngle()).getCos() * Constants.WRIST_KFF);
  }
  /**
   * Sets the percent without calculating feedforward
   * @param percent [-1.0.. 1.0]
   */
  public void setPercent(double percent)
  {
    srx.set(percent);
  }
  @Override
  public void periodic()
  {
  }
}

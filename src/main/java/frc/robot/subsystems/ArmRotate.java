// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.variants.MotorGroupTalon;

import com.ctre.phoenix.sensors.CANCoder;

public class ArmRotate extends ProfiledPIDSubsystem {
  // We know we will have two talons
  private final MotorGroupTalon talonGroup;
  private final CANCoder rotateEncoder;
  private final ArmFeedforward feedforward = new ArmFeedforward(Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV);
  /** Creates a new Arm. */
  public ArmRotate() {
    super(
      new ProfiledPIDController(
        Constants.ARM_PROPORTION,
        0,
        0,
        new Constraints(Constants.ARM_MAX_RADIANS_PER_SECOND, Constants.ARM_MAX_RADIANS_PER_SECOND_SQUARED)
      )
    );
    talonGroup = new MotorGroupTalon(Constants.ARM_PRIMARY_MOTOR_ID, new int[]
    {
      Constants.ARM_FOLLOWER_MOTOR_ID
    });
    talonGroup.setFollowerOppose(0);
    rotateEncoder = new CANCoder(Constants.ARM_ROTATE_CANCODER_ID);
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
    setGoal(angle.getRadians());
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
    setGoal(getController().getGoal().position + Rotation2d.fromDegrees(speed).getRadians());
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", getAngle().getDegrees());
  }
  @Override
  protected void useOutput(double output, State setpoint) {
    talonGroup.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
  }
  @Override
  protected double getMeasurement() {
    return getAngle().getRadians();
  }
}
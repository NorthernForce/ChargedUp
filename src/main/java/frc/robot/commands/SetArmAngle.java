// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.*;

public class SetArmAngle extends CommandBase {
  private final Rotation2d angle;
  /** Creates a new SetArmAngle. */
  public SetArmAngle(Rotation2d angle) {
    this.angle = Rotation2d.fromDegrees(MathUtil.inputModulus(angle.getDegrees(), -90, 270));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armRotate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("armAngle", angle.getRotations() * 360);
    SmartDashboard.putNumber("armAngle (getDegrees)", angle.getDegrees());
    armRotate.setAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armRotate.getAngle().minus(angle).getDegrees()) < Constants.ArmConstants.ANGLE_TOLERANCE
      && Math.abs(armRotate.getVelocity().getDegrees()) < Constants.ArmConstants.ANGLE_TOLERANCE;
  }
}

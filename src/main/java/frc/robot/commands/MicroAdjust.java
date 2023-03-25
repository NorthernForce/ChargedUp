// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OI;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.navigation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MicroAdjust extends CommandBase {
  private double rotationSpeed;
  private Rotation2d startAngle;
  /** Creates a new MicroAdjust. */
  public MicroAdjust(double rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.rotationSpeed = rotationSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startAngle = (navigation.getPose2d().getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0,rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Rotation2d currentAngle = navigation.getPose2d().getRotation();
    return Math.abs(currentAngle.minus(this.startAngle).getDegrees()) > Constants.DrivetrainConstants.MICRO_ADJUST_DEGREES.getDegrees();
  }
}

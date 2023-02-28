// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import static frc.robot.RobotContainer.drivetrain;

public class Brake extends CommandBase {
  private double vx;
  /** Creates a new Brake. */
  public Brake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vx = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DifferentialDriveWheelSpeeds speeds = drivetrain.getSpeeds();
    vx = (speeds.leftMetersPerSecond + speeds.rightMetersPerSecond) / 2; //Speed of the robot forward
    drivetrain.driveUsingChassisSpeeds(new ChassisSpeeds(-vx,0,0));

    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("Left MPS", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right MPS", speeds.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

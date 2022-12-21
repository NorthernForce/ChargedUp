// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.RobotContainer.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.trackingSystem;

public class FollowTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PIDController forwardController, turnController;
  private final double distance;
  public FollowTarget(double distance) {
    addRequirements(drivetrain, trackingSystem);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double forwardP = SmartDashboard.getNumber("Forward Controller - kP", 0.2);
    double turnP = SmartDashboard.getNumber("Turn Controller - kP", 0.12);
    double forwardI = SmartDashboard.getNumber("Forward Controller - kI", 0.2);
    double turnI = SmartDashboard.getNumber("Turn Controller - kI", 0);
    double forwardD = SmartDashboard.getNumber("Forward Controller - kD", 0);
    double turnD = SmartDashboard.getNumber("Turn Controller - kD", 0);
    turnController = new PIDController(turnP, turnI, turnD);
    turnController.setSetpoint(0);
    forwardController = new PIDController(forwardP, forwardI, forwardD);
    forwardController.setTolerance(0.2);
    forwardController.setSetpoint(distance);
    SmartDashboard.putNumber("Forward Controller - kP", forwardP);
    SmartDashboard.putNumber("Turn Controller - kP", turnP);
    SmartDashboard.putNumber("Forward Controller - kI", forwardI);
    SmartDashboard.putNumber("Turn Controller - kI", turnI);
    SmartDashboard.putNumber("Forward Controller - kD", forwardD);
    SmartDashboard.putNumber("Turn Controller - kD", turnD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forwardController.setP(SmartDashboard.getNumber("Forward Controller - kP", 0.2));
    turnController.setP(SmartDashboard.getNumber("Turn Controller - kP", 0.12));
    forwardController.setI(SmartDashboard.getNumber("Forward Controller - kI", 0.2));
    turnController.setI(SmartDashboard.getNumber("Turn Controller - kI", 0));
    forwardController.setD(SmartDashboard.getNumber("Forward Controller - kD", 0));
    turnController.setD(SmartDashboard.getNumber("Turn Controller - kD", 0));
    double rotate = 0;
    double xSpeed = 0;
    if (trackingSystem.hasTargets())
    {
      double range = trackingSystem.estimateRangeMeters();
      if (Math.abs(trackingSystem.getTargetYawDegrees()) >= 1) rotate = turnController.calculate(trackingSystem.getTargetYawDegrees());
      if (Math.abs(range - distance) >= 0.2) xSpeed = -forwardController.calculate(range);
      SmartDashboard.putNumber("Forward speed", xSpeed);
      SmartDashboard.putNumber("Range", range);
    }
    drivetrain.drive(xSpeed, rotate);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
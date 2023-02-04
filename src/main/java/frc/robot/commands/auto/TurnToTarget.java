// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class TurnToTarget extends CommandBase {
  /** Creates a new TurnToTarget. */
  private PIDController calculator;
  private PIDController calculator2;

  public TurnToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculator = new PIDController(5e-4, 0, 0);
    calculator.setSetpoint(0);
    calculator2 = new PIDController(0.2, 0, 0);
    calculator2.setSetpoint(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasTarget()){
      double requiredTurn = -calculator.calculate(Math.toDegrees(vision.getTargetYaw()));
      SmartDashboard.putNumber("Range", vision.getTargetRange());
      drivetrain.drive(calculator2.calculate(vision.getTargetRange()), requiredTurn);
    }
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
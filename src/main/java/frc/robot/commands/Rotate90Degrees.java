// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Rotate90Degrees extends CommandBase {
  private PIDController controller;
  private double speed;

  /** Creates a new Rotate90Degrees.
   * @param speed make this positive or negative until it works (writing this code at 11:45pm)
  */
  public Rotate90Degrees(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.imu, RobotContainer.drivetrain);
    controller = new PIDController(0.00555, 1e-5, 0);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(5);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    controller.setSetpoint(DriverStation.getAlliance() == Alliance.Red ? 90 : -90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivetrain.drive(0, speed * controller.calculate(RobotContainer.imu.getYaw().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}

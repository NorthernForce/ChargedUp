// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.*;

public class DriveMeters extends CommandBase {
  private final double speed, rotation, meters;
  private final PIDController alignController = new PIDController(0.04, 0, 0);
  private double startMeters;
  /** Creates a new DriveForDistance. */
  public DriveMeters(double speed, double meters) {
    addRequirements(drivetrain);
    this.speed = speed;
    this.rotation = imu.getYaw();
    this.meters = meters;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startMeters = (drivetrain.getRightDistance() + drivetrain.getLeftDistance())/2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(speed, -alignController.calculate(imu.getYaw(), rotation));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double currentMeters = (drivetrain.getRightDistance() + drivetrain.getLeftDistance())/2;
    return (Math.abs(currentMeters - this.startMeters) > meters);
  }
}
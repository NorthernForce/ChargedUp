// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.drivetrain;;

public class DriveMeters extends CommandBase {
  private final double speed, rotation, meters;
  /** Creates a new DriveForDistance. */
  public DriveMeters(double speed, double rotation, double meters) {
    addRequirements(drivetrain);
    this.speed = speed;
    this.rotation = rotation;
    this.meters = meters;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getRightDistance() + drivetrain.getLeftDistance())/2 > meters);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.*;

public class DriveForSeconds extends CommandBase {
  private final double speed;
  private final double rotation;
  private final double seconds;
  private final double startTime;
  /** 
   * Creates a new DriveForMilliseconds.
   * @param speed forward speed [1.0.. -1.0]
   * @param rotation rotational speed [1.0.. -1.0]
   * @param milliseconds duration of command [0.0.. 1.84e19]
  */
  public DriveForSeconds(double speed, double rotation, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.speed = speed;
    this.rotation = rotation;
    this.seconds = seconds;
    this.startTime = Timer.getFPGATimestamp();
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
    double nowMilli = Timer.getFPGATimestamp();
    return ((nowMilli - startTime) >= seconds);
  }
}
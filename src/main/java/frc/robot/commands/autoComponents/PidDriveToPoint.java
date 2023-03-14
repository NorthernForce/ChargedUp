// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.autoComponents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.navigation;

public class PidDriveToPoint extends CommandBase {
  private final double speed;
  private final double stopTolerance;
  private long startTime;
  private long maxduration = 10000;
  private Pose2d target = new Pose2d(0, 0 , Rotation2d.fromDegrees(0));
  private double distanceToGo = 0;
  // PID constants should be tuned per robot

  final double LINEAR_P = 1.0;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  final double ANGULAR_P = 5e-2;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param drive The drivetrain subsystem on which this command will run
   */
  public PidDriveToPoint(Pose2d target, double speed, double stopTolerance) {
    SmartDashboard.putString("DriveToTagStatus", "Constructed");
    this.speed = speed;
    this.stopTolerance = stopTolerance;
    this.target = target;
    addRequirements(drivetrain, navigation);
    SmartDashboard.putData(forwardController);
    SmartDashboard.putData(turnController);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    drivetrain.drive(0, 0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = 0;
    double rotationSpeed = 0;

    // Use this range as the measurement we give to the PID controller.
    double distanceToGo = target.getTranslation().getDistance(navigation.getPose2d().getTranslation());

    // -1.0 required to ensure positive PID controller effort _increases_ range
    forwardSpeed = -forwardController.calculate(distanceToGo, 0);

    // Also calculate angular power
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    double yawToTarget = target.getRotation().minus(navigation.getPose2d().getRotation()).getDegrees();
    yawToTarget = MathUtil.inputModulus(yawToTarget, -180, 180);
    rotationSpeed = -turnController.calculate(yawToTarget, 0);

    drivetrain.drive(forwardSpeed * speed, rotationSpeed * speed);
    SmartDashboard.putString("DriveToTag", "EndingExecute");
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceToGo < stopTolerance) {
      return true;
    }
    if ((System.currentTimeMillis() - startTime) > maxduration) {
      return true;
    }
    return false;
  }
}
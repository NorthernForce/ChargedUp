// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.autoComponents;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


import static frc.robot.RobotContainer.*;


public class TurnToCoordinates extends CommandBase {
  private final Translation2d coords;
  private final double speed;
  private Rotation2d targetRotation;
  /**
   * Creates a new TurnToCoordinates.
   * @param coords The coordinates of the thing to align with
   * @param speed The speed to turn at
   */
  public TurnToCoordinates(Translation2d coords, double speed) {
    addRequirements(drivetrain, navigation);
    this.coords = coords;
    this.speed = speed;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetRotation = navigation.getPose2d().getTranslation().minus(coords).getAngle();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0, targetRotation.minus(navigation.getPose2d().getRotation()).getDegrees() > 0 ? -speed : speed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetRotation.minus(navigation.getPose2d().getRotation()).getDegrees()) < 2;
  }
}
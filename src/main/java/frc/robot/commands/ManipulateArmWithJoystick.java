// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

import static frc.robot.RobotContainer.*;

import java.util.function.DoubleSupplier;

public class ManipulateArmWithJoystick extends CommandBase {
  private DoubleSupplier[] Speeds = OI.getManipulatorSuppliers();
  /** Creates a new ManipulateWithJoystick. */
  public ManipulateArmWithJoystick() {
    addRequirements(armRotate);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //armRotate.setArmVoltage(armRotate.getFeedforward().calculate(armRotate.getAngle().getRadians(), Speeds[0].getAsDouble()));
    armRotate.setArmPosition(Rotation2d.fromDegrees(0));
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

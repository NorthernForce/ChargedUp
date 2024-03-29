// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

import static frc.robot.RobotContainer.*;

import java.util.function.DoubleSupplier;

public class ManipulateArmWithJoystick extends CommandBase {
  private DoubleSupplier[] Speeds = OI.getManipulatorSuppliers();
  /** Creates a new ManipulateWithJoystick. */
  public ManipulateArmWithJoystick() {
    addRequirements(armRotate);
    Shuffleboard.getTab("Arm").addNumber("Arm input", () -> Speeds[0].getAsDouble());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armRotate.setArmSpeed(Speeds[0].getAsDouble() * Math.abs(Speeds[0].getAsDouble()));
    if (armRotate.getAngle().getDegrees() > 56 && armRotate.getAngle().getDegrees() < 124)
    {
      armTelescope.retract();
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static frc.robot.RobotContainer.armRotate;
import static frc.robot.RobotContainer.wrist;

public class PositionManipulatorForSingleSubstation extends CommandBase {
  private GenericEntry armSingleSubstationEntry;
  private GenericEntry wristSingleSubstationEntry;
  /** Creates a new PositionManipulatorForSingleSubstation. */
  public PositionManipulatorForSingleSubstation() {
    addRequirements(armRotate, wrist);
    armSingleSubstationEntry = Shuffleboard.getTab("Arm").add("Arm Single Angle", Constants.ArmConstants.SINGLE_SUBSTATION_ANGLE.getDegrees()).getEntry();
    wristSingleSubstationEntry = Shuffleboard.getTab("Arm").add("Wrist Single Angle", Constants.WristConstants.SINGLE_SUBSTATION_ANGLE.getDegrees()).getEntry();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armRotate.setAngle(Rotation2d.fromDegrees(armSingleSubstationEntry.getDouble(Constants.ArmConstants.SINGLE_SUBSTATION_ANGLE.getDegrees())));
    wrist.setRotation(Rotation2d.fromDegrees(wristSingleSubstationEntry.getDouble(Constants.WristConstants.SINGLE_SUBSTATION_ANGLE.getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armRotate.setArmSpeed(0);
    wrist.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

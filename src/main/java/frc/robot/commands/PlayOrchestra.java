// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Motors.MotorGroupTalonFX;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class PlayOrchestra extends CommandBase {
  private Orchestra orchestra;
  private GenericEntry isRunning;
  /** Creates a new PlayOrchestra. */
  public PlayOrchestra(String songName, MotorGroupTalonFX... motors) {
    orchestra = new Orchestra();
    for (var motor : motors)
    {
      for (var controller : motor.getAllMotors())
      {
        orchestra.addInstrument(controller);
      }
    }
    Shuffleboard.getTab("utilities").addBoolean("isPlaying", orchestra::isPlaying);
    Shuffleboard.getTab("utilities").addDouble("time", orchestra::getCurrentTime);
    isRunning = Shuffleboard.getTab("utilities").add("isRunning", false).getEntry();
    orchestra.loadMusic(songName + ".chrp");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRunning.setBoolean(true);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    orchestra.play();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
    isRunning.setBoolean(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

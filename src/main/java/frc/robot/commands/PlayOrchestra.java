// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Motors.MotorGroupTalonFX;

public class PlayOrchestra extends CommandBase {
  private Orchestra orchestra;
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
    orchestra.loadMusic(songName + ".chirp");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    orchestra.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

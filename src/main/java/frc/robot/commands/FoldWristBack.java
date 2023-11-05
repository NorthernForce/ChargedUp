// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FoldWristBack extends ParallelDeadlineGroup {
  /** Creates a new FoldWristBack. */
  public FoldWristBack() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(0.5), Commands.run(() -> wrist.setPercent(-0.8), wrist));
    // addCommands(new FooCommand(), new BarCommand());
  }
}

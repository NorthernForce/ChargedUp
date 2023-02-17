// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoComponents.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_S_Eng extends SequentialCommandGroup {
  /** Creates a new CG_S_Eng. */
  public CG_S_Eng() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Backup
      new DriveMeters(-0.3, 0, 0.3),
      //Push piece of front
      new DriveMeters(0.3, 0, 0.05),
      new DriveMeters(0.55, 0, 0.05),
      new DriveMeters(-0.55, 0, 0.2),
      //Push piece forward
      new DriveMeters(0.35, 0, 0.4),
      new DriveForMilliseconds(.3, 0, 5500),
      //Exit Community
      new DriveMeters(-0.55, 0, 1),
      new DriveMeters(-0.35, 0, 0.6),
      new DriveMeters(-0.55, 0, 1),
      //Stop
      new Stop(0.1));
  }
}

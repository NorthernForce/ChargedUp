// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoComponents.DriveForMilliseconds;
import frc.robot.commands.autoComponents.DriveMeters;
import frc.robot.commands.autoComponents.Stop;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OG_1PieMob extends SequentialCommandGroup {
  /** Creates a new HG_1PieMob. */
  public OG_1PieMob() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    //Backup
    addCommands(new DriveMeters(-0.3, 0, 0.3));
    //Push piece off front
    addCommands(new DriveMeters(0.3, 0, 0.05));
    addCommands(new DriveMeters(0.55, 0, 0.05));
    addCommands(new DriveMeters(-0.55, 0, 0.2));
    //Push piece forward
    addCommands(new DriveMeters(0.35, 0, 0.4));
    addCommands(new DriveForMilliseconds(.3, 0, 5500));
    //Exit Community
    addCommands(new DriveMeters(-0.55, 0, 1));
    addCommands(new DriveMeters(-0.35, 0, 0.6));
    addCommands(new DriveMeters(-0.55, 0, 1));

    addCommands(new Stop(0.1));
  }
}
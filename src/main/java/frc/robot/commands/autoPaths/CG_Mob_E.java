// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoComponents.Balance;
import frc.robot.commands.autoComponents.DriveForMilliseconds;
import frc.robot.commands.autoComponents.DriveForSeconds;
import frc.robot.commands.autoComponents.DriveMeters;
import frc.robot.commands.autoComponents.Stop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_Mob_E extends SequentialCommandGroup {
  /** Creates a new CG_Mob_E. */
  public CG_Mob_E() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveMeters(-0.3, 0, 0.3),
      //Push piece of front
      new DriveMeters(0.3, 0, 0.05),
      new DriveMeters(0.55, 0, 0.05),
      new DriveMeters(-0.55, 0, 0.2),
      //Push piece forward
      new DriveMeters(0.35, 0, 0.5),
      //Exit Community
      new DriveMeters(-0.75, 0, 0.6),
      new DriveMeters(-0.5, 0, 1.5),
      new DriveMeters(-0.4, 0, 1),
      // Reenter Charge Station
      new DriveMeters(0.7, 0, 2.3),
      new Stop(0.05),
      new Balance()
    );
  }
}

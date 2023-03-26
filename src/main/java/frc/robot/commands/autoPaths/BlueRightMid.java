// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPaths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.FoldWristBack;
import frc.robot.commands.ManipulateCube;
import frc.robot.commands.autoComponents.DriveMeters;
import frc.robot.commands.autoComponents.PositionWithTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueRightMid extends SequentialCommandGroup {
  /** Creates a new BlueLeftMid. */
  public BlueRightMid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ManipulateCube(),
      new FoldWristBack(),
      new PositionWithTarget(FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS[0].toTranslation2d(), Constants.AnglesAndDistances.MEDIUM_CUBE.getFirst(), Constants.AnglesAndDistances.MEDIUM_CUBE.getSecond(), Rotation2d.fromDegrees(0), false),
      new DriveMeters(-0.5, 0, 5)
    );
  }
}

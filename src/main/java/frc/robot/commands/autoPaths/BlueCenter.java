// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPaths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.Outtake;
import frc.robot.commands.RetractArm;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.autoComponents.Balance;
import frc.robot.commands.autoComponents.DriveMeters;
import frc.robot.commands.autoComponents.PositionWithTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueCenter extends SequentialCommandGroup {
  /** Creates a new BlueCenter. */
  public BlueCenter() {
    var placingInformation = Constants.calculateArmAngleAndDistance(Constants.ArmConstants.ORIGIN.getZ(), Constants.ArmConstants.EXTENDED_LENGTH, Units.inchesToMeters(9), Rotation2d.fromDegrees(0), FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[3].getZ(), Constants.ArmConstants.ORIGIN.getX(), Rotation2d.fromDegrees(19));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PositionWithTarget(FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[3].toTranslation2d(), placingInformation.getFirst(), placingInformation.getSecond(), Rotation2d.fromDegrees(10), true),
      new ParallelDeadlineGroup(new WaitCommand(0.8), new Outtake()),
      new RetractArm(),
      new ParallelCommandGroup(new SetArmAngle(Rotation2d.fromDegrees(60)),
      new DriveMeters(-0.55, 0, 1.5)),
      new ParallelCommandGroup(//new SetArmAngle(Rotation2d.fromDegrees(180)),
      new DriveMeters(-0.6, 0, 0.2)),
      new ParallelCommandGroup(//new SetArmAngle(Rotation2d.fromDegrees(90)),
      new Balance())
    );
  }
}

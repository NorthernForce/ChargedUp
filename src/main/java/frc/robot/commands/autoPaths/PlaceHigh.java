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
import frc.robot.commands.ExtendArm;
import frc.robot.commands.FoldWristBack;
import frc.robot.commands.Outtake;
import frc.robot.commands.SetArmAngle;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHigh extends SequentialCommandGroup {
  /** Creates a new PlaceHigh. */
  public PlaceHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var placingInformation = Constants.calculateArmAngleAndDistance(Constants.ArmConstants.ORIGIN.getZ(), Constants.ArmConstants.EXTENDED_LENGTH, Units.inchesToMeters(9), Rotation2d.fromDegrees(0), FieldConstants.RED_CUBE_PLACEMENT_LOCATIONS[3].getZ(), Constants.ArmConstants.ORIGIN.getX());
    addCommands(
      new ParallelCommandGroup(new FoldWristBack(), new SetArmAngle(placingInformation.getSecond())),
      new ExtendArm(),
      new ParallelDeadlineGroup(new WaitCommand(0.8), new Outtake())
    );
  }
}
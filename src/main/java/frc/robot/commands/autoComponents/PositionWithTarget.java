// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RetractArm;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetWristAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionWithTarget extends ParallelCommandGroup {
  /** Creates a new PositionWithTarget. */
  public PositionWithTarget(Translation2d targetPosition,
    double targetDistance,
    Rotation2d targetArmAngle,
    Rotation2d targetWristAngle,
    boolean extendArm) {
    this(targetPosition, targetDistance, targetArmAngle, targetWristAngle, extendArm, false);
  }
  public PositionWithTarget(Translation2d targetPosition,
    double targetDistance,
    Rotation2d targetArmAngle,
    Rotation2d targetWristAngle,
    boolean extendArm, boolean reversed) {
    addCommands(
      new SequentialCommandGroup(
        //new TurnToCoordinates(targetPosition, reversed)
        //new DriveDistanceFromCoordinates(targetDistance, targetPosition, reversed)
      ),
      new SetArmAngle(targetArmAngle),
      new SetWristAngle(targetWristAngle),
      extendArm ? new AutoExtend() : new RetractArm()
    );
  }
}

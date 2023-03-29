// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPaths;

import java.io.IOException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.AnglesAndDistances;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.FoldWristBack;
import frc.robot.commands.Intake;
import frc.robot.commands.ManipulateCube;
import frc.robot.commands.Outtake;
import frc.robot.commands.RetractArm;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.autoComponents.DriveAlongPath;
import frc.robot.commands.autoComponents.DriveMeters;
import frc.robot.commands.autoComponents.PositionWithTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueRight extends SequentialCommandGroup {
  /** Creates a new BlueLeft. 
   * @throws IOException */
  public BlueRight(int numPieces) throws IOException {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FoldWristBack(),
      new ManipulateCube(),
      new PositionWithTarget(FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[1].toTranslation2d(), AnglesAndDistances.HIGH_CUBE.getFirst(),
        AnglesAndDistances.HIGH_CUBE.getSecond().plus(Rotation2d.fromDegrees(20)),
        WristConstants.HIGH_CUBE_PLACEMENT_ANGLE, true),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new Outtake()
      ),
      new RetractArm(),
      new SetArmAngle(Rotation2d.fromDegrees(90)),
      new DriveMeters(-0.4, 0, 1),
      new DriveAlongPath(Constants.Path.BACKWARD_BLUE_RIGHT_TO_PIECE_RIGHT)
    );
    if (numPieces > 1)
    {
      addCommands(
        new PositionWithTarget(FieldConstants.BLUE_GAME_PIECE_AUTO_LOCATIONS[0].toTranslation2d(), AnglesAndDistances.BACKWARD_FLOOR_CUBE.getFirst(),
          AnglesAndDistances.BACKWARD_FLOOR_CUBE.getSecond().plus(Rotation2d.fromDegrees(20)),
          WristConstants.BACKWARD_PICKUP_ANGLE, true, true),
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new Intake()
        ),
        new RetractArm(),
        new SetArmAngle(Rotation2d.fromDegrees(90)),
        new DriveAlongPath(Constants.Path.FORWARD_PIECE_RIGHT_TO_BLUE_RIGHT),
        new PositionWithTarget(FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[0].toTranslation2d(), AnglesAndDistances.MEDIUM_CUBE.getFirst(),
          AnglesAndDistances.MEDIUM_CUBE.getSecond().plus(Rotation2d.fromDegrees(20)),
          WristConstants.MID_CUBE_PLACEMENT_ANGLE, false),
        new ParallelDeadlineGroup(
          new WaitCommand(0.5),
          new Outtake()
        ),
        new SetArmAngle(Rotation2d.fromDegrees(90))
      );
    }
    if (numPieces > 2)
    {
      addCommands(
        new DriveMeters(-0.4, 0, 1),
        new DriveAlongPath("Blue1ToPiece2"),
        new PositionWithTarget(FieldConstants.BLUE_GAME_PIECE_AUTO_LOCATIONS[1].toTranslation2d(), AnglesAndDistances.LOW_CUBE.getFirst(),
          AnglesAndDistances.LOW_CUBE.getSecond().plus(Rotation2d.fromDegrees(20)),
          WristConstants.LOW_CUBE_PLACEMENT_ANGLE, false),
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new Intake()
        ),
        new SetArmAngle(Rotation2d.fromDegrees(90)),
        new DriveAlongPath("Piece2ToBlue1"),
        new PositionWithTarget(FieldConstants.BLUE_FLOOR_PLACEMENT_LOCATIONS[1].toTranslation2d(), AnglesAndDistances.LOW_CUBE.getFirst(),
          AnglesAndDistances.LOW_CUBE.getSecond().plus(Rotation2d.fromDegrees(20)),
          WristConstants.LOW_CUBE_PLACEMENT_ANGLE, false),
        new ParallelDeadlineGroup(
          new WaitCommand(0.5),
          new Outtake()
        )
      );
    }
  }
}

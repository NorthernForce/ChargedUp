// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPaths;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.RetractArm;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.autoComponents.PidDriveToPoint;
import frc.robot.commands.autoComponents.PositionWithTarget;
import static frc.robot.RobotContainer.manipulatingState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlideLoadingStation extends SequentialCommandGroup {
  /** Creates a new SlideLoadingStation. */
  public SlideLoadingStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Pair<Double, Rotation2d> distanceAngle = manipulatingState.getCurrentState().getSlideApproachPosition();

    addCommands(
      new PidDriveToPoint(FieldConstants.AutoLocations.SLIDE_GRID_DISTANCE, 1, 1),
      new PositionWithTarget(
        FieldConstants.AutoLocations.SLIDE_GRID.getTranslation(), 
        distanceAngle.getFirst(), 
        distanceAngle.getSecond(), 
        manipulatingState.getCurrentState().getSlideWristAngle(), 
        true),
      new Intake().raceWith(
        new SetArmAngle(
          manipulatingState.getCurrentState().getSlidePickUpPosition().getSecond()
          )
        )
      // new Intake().raceWith(new SetArmAngle(Rotation2d.fromDegrees(90))), // ARM UP
      // new RetractArm()
    );
  }
}
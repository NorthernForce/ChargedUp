// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.autoComponents;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.robot.RobotContainer.*;


public class TurnToCoordinates extends PIDCommand {
  /**
   * Creates a new TurnToCoordinates.
   * @param coords The coordinates of the thing to align with
   * @param speed The speed to turn at
   */
  public TurnToCoordinates(Translation2d coords, double speed) {
    super(
      new PIDController(
        2e-2,
        2e-3,
        0
      ),
      () -> navigation.getPose2d().getTranslation().minus(coords).getAngle().getDegrees(),
      () -> navigation.getPose2d().getRotation().getDegrees(),
      (output) -> drivetrain.drive(0, output),
      drivetrain, navigation);
  }
  @Override
  public boolean isFinished()
  {
    return getController().atSetpoint();
  }
}
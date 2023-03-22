// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.autoComponents;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.robot.RobotContainer.*;

import javax.lang.model.util.ElementScanner14;


public class TurnToCoordinates extends PIDCommand {
  /**
   * Creates a new TurnToCoordinates.
   * @param coords The coordinates of the thing to align with
   */
  public TurnToCoordinates(Translation2d coords) {
    super(
      new PIDController(
        2e-2,
        0,
        0
      ),
      () -> {
        var val = -coords.minus(navigation.getPose2d().getTranslation()).getAngle().getDegrees() - navigation.getPose2d().getRotation().getDegrees();
        if (val > 180)
        {
          val -= 360;
        }
        else if (val < -180)
        {
          val += 360;
        }
        return val;
      },
      () -> navigation.getPose2d().getRotation().getDegrees(),
      (output) -> drivetrain.drive(0, -output),
      drivetrain, navigation);
      getController().setTolerance(8);
  }
  @Override
  public boolean isFinished()
  {
    return getController().atSetpoint();
  }
}
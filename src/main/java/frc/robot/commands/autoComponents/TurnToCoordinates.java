// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.autoComponents;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.robot.RobotContainer.*;

public class TurnToCoordinates extends PIDCommand {
  /**
   * Creates a new TurnToCoordinates.
   * @param coords The coordinates of the thing to align with
   */
  public TurnToCoordinates(Translation2d coords)
  {
    this(coords, false);
  }
  public TurnToCoordinates(Translation2d coords, boolean reversed) {
    super(
      new PIDController(
        2e-2,
        0,
        0
      ),
      () -> {
        if (reversed)
        {
          var val = -navigation.getPose2d().getTranslation().minus(coords).getAngle().getDegrees() - navigation.getPose2d().getRotation().getDegrees();
          return MathUtil.inputModulus(val, -180.0, 180.0);
        }
        var val = -coords.minus(navigation.getPose2d().getTranslation()).getAngle().getDegrees() - navigation.getPose2d().getRotation().getDegrees();
        return MathUtil.inputModulus(val, -180.0, 180.0);
      },
      () -> 0,
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
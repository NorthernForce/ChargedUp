// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GripperConstants;
import frc.robot.commands.Outtake;
import frc.robot.commands.RunConeIntake;
import frc.robot.commands.RunConeOuttake;
import frc.robot.commands.SetArmAngle;
import frc.robot.subsystems.ArmRotate;

import static frc.robot.Constants.WristConstants.*;
import static frc.robot.RobotContainer.armRotate;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoOuttake extends SequentialCommandGroup {
  /** Creates a new AutoOuttake. 
   * assumes cube
  */
  public AutoOuttake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmAngle(armRotate.getAngle().minus(CUBE_OVERSHOOT)),
      new ParallelDeadlineGroup(Commands.waitSeconds(GripperConstants.INTAKE_TIME), new Outtake())
    );
  }
}

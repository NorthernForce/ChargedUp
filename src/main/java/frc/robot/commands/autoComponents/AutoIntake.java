// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.RobotContainer.armRotate;
import frc.robot.Constants.GripperConstants;
import static frc.robot.Constants.WristConstants.*;
import frc.robot.commands.RunConeIntake;
import frc.robot.commands.RunConeOuttake;
import frc.robot.commands.SetArmAngle;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends SequentialCommandGroup {
  /** Creates a new Intake. 
   * intakes a cone
  */
  public AutoIntake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(Commands.waitSeconds(GripperConstants.INTAKE_TIME), new RunConeOuttake(), new SetArmAngle(armRotate.getAngle().minus(CUBE_DROPDOWN.plus(CUBE_OVERSHOOT)))
    ));
  }
  /** Creates a new Intake. 
   * @param isCube is the pice a cone?
  */
  public AutoIntake(boolean isCube) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(Commands.waitSeconds(GripperConstants.INTAKE_TIME), isCube ? new RunConeOuttake() : new RunConeIntake(), isCube ? new SetArmAngle(armRotate.getAngle().minus(CUBE_DROPDOWN.plus(CUBE_OVERSHOOT))) : new SetArmAngle(armRotate.getAngle().minus(CONE_DROPDOWN.plus(CONE_OVERSHOOT)))
    ));
  }
}

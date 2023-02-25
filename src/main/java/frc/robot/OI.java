// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.RunConeIntake;
import frc.robot.commands.RunConeOuttake;
import frc.robot.commands.SlowMode;
import frc.robot.commands.LEDPurple;
import frc.robot.commands.LEDYellow;
import frc.robot.commands.RetractArm;

/** Add your docs here. */
public class OI {
    private static final XboxController driverController = new XboxController(0);
    private static final XboxController manipulatorController = new XboxController(1);
    public OI() {}
    /**
     * Gets the joystick controls from the drivercontroller we use to control the robots drivetrain
     * @return Index[0]: Joystick forward(positve)/back(negative).
     * <li> Index[1]: Joystick left(positive)/right(negative). </li>
     */
    public static DoubleSupplier[] getDriveSuppliers() {
            return new DoubleSupplier[] {
            //Joystick defaults to forward being negative so we negate it.
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()
        };
    }
    public static DoubleSupplier[] getManipulatorSuppliers() {
            return new DoubleSupplier[] {
            () -> -manipulatorController.getLeftY(),
            () -> manipulatorController.getRightX()
        };
    }
    public void bindButtons() {
        if (Constants.DRIVETRAIN_ENABLED)
        {
            //new JoystickButton(driverController, XboxController.Button.kA.value)
              //  .toggleOnTrue(new SlowMode());
        }
        if (Constants.ARM_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Axis.kRightTrigger.value)
                .whileTrue(new ExtendArm());
            new JoystickButton(manipulatorController, XboxController.Axis.kRightTrigger.value)
                .whileFalse(new RetractArm());
        }
        if (Constants.GRIPPER_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Button.kA.value)
                .onTrue(new RunConeIntake());
            new JoystickButton(manipulatorController, XboxController.Button.kB.value)
                .onTrue(new RunConeOuttake());
        }
        if (Constants.LED_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Button.kY.value)
                .whileTrue(new LEDPurple());
        }
        if (Constants.LED_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Button.kX.value)
                .whileTrue(new LEDYellow());
        }
    }
}

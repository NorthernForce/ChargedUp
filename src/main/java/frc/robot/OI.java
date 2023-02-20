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
import frc.robot.commands.SwitchLED;

/** Add your docs here. */
public class OI {
    private static final XboxController driverController = new XboxController(0);
    private static final XboxController manipulatorController = new XboxController(1);
    /**
     * Gets two double suppliers representing the two major axis of control on drive controller.
     * @return two doubles that will be within [-1.0, 1.0]
     */
    public static DoubleSupplier[] getDriveSuppliers() {
            return new DoubleSupplier[] {
            () -> -driverController.getLeftY(),
            () -> driverController.getRightX()
        };
    }
    /**
     * Gets two double suppliers representing the two major axis of control on manipulator controller.
     * @return two doubles that will be within [-1.0, 1.0]
     */
    public static DoubleSupplier[] getManipulatorSuppliers() {
            return new DoubleSupplier[] {
            () -> -manipulatorController.getLeftY(),
            () -> manipulatorController.getRightX()
        };
    }
    /** Binds the buttons of the OI */
    public void bindButtons() {
        if (Constants.ARM_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Axis.kRightTrigger.value)
                .whileTrue(new ExtendArm());
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
                .onTrue(new SwitchLED());
        }
    }
}

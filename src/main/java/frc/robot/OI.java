// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SlowMode;

/** Add your docs here. */
public class OI {
    private static final XboxController driverController = new XboxController(0);
    private static final XboxController manipulatorController = new XboxController(1);
    public OI() {}
    public static DoubleSupplier[] getDriveSuppliers() {
            return new DoubleSupplier[] {
            () -> -driverController.getLeftY(),
            () -> driverController.getRightX()
        };
    }

    public void bindButtons() {
        new JoystickButton(driverController, XboxController.Button.kA.value)
            .toggleOnTrue(new SlowMode());
    }
}

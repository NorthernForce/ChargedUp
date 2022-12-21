// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
    private static final XboxController driverController = new XboxController(0);

    public OI() {
        // intake = new Intake();
    }

    public static DoubleSupplier[] getDriveSuppliers() {
       return new DoubleSupplier[] {() -> driverController.getLeftY(), () -> -driverController.getRightX()};
    }

    public void bindButtons() {
    }
}

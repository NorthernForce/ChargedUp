// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.variants.DrivetrainSquishy;

/** Chassis specific details about squishy. */
public class SquishyChassis implements ChassisBase {
    public SquishyChassis() {}
    public Drivetrain getDrivetrain() {
        return new DrivetrainSquishy();
    }
    public double getConstant(String key) {
        return 0;
    }
    public String getChassisName() {
        return "Squishy";
    }
}
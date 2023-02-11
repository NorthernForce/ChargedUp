// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.variants.DrivetrainSpeedy;

/** Add your docs here. */
public class SpeedyChassis implements ChassisBase {
    public SpeedyChassis() {}
    public Drivetrain getDrivetrain() {
        return new DrivetrainSpeedy();
    }
    public double getConstant(String key) {
        return 0;
    }
    public String getChassisName() {
        return "Speedy";
    }
}
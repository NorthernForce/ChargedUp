// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import frc.robot.subsystems.Drivetrain;

/** This is ChassisBase. It is an interface that allows you to get chassis specifics */
public interface ChassisBase {
    public Drivetrain getDrivetrain();
    public double getConstant(String key);
    public String getChassisName();
}

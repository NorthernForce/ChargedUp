// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MotorGroup;
import frc.robot.subsystems.variants.MotorGroupTalon;

/** Chassis specific details about speedy. */
public class SpeedyChassis implements ChassisBase {
    public SpeedyChassis() {}
    public Drivetrain getDrivetrain() {
        MotorGroup left = new MotorGroupTalon(LEFT_PRIMARY_ID, new int[]{LEFT_FOLLOWER_ID});
        MotorGroup right = new MotorGroupTalon(RIGHT_PRIMARY_ID, new int[]{RIGHT_FOLLOWER_ID});
        right.setInverted(true);
        return new Drivetrain(left, right);
    }
    public String getChassisName() {
        return "Speedy";
    }
    public static final int LEFT_PRIMARY_ID = 2;
    public static final int RIGHT_PRIMARY_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 3;
}
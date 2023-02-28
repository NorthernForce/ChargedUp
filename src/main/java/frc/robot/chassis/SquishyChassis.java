// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MotorGroup;
import frc.robot.subsystems.variants.MotorGroupSpark;

/** Chassis specific details about squishy. */
public class SquishyChassis implements ChassisBase {
    public SquishyChassis() {}
    public Drivetrain getDrivetrain() {
        MotorGroup left = new MotorGroupSpark(MotorType.kBrushless, LEFT_PRIMARY_ID, LEFT_FOLLOWER_ID);
        left.setInverted(true);
        MotorGroup right = new MotorGroupSpark(MotorType.kBrushless, RIGHT_PRIMARY_ID, RIGHT_FOLLOWER_ID);
        return new Drivetrain(left, right);
    }
    public String getChassisName() {
        return "Squishy";
    }
    public static final int LEFT_PRIMARY_ID = 1;
    public static final int RIGHT_PRIMARY_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;
}
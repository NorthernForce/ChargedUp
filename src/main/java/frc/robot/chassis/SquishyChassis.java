// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MotorGroup;
import frc.robot.subsystems.variants.MotorGroupSpark;

/** Chassis specific details about squishy. */
public class SquishyChassis implements ChassisBase {
    public static final double TRACK_WIDTH = Units.inchesToMeters(24);
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public SquishyChassis() {}
    public Drivetrain getDrivetrain() {
        MotorGroup left = new MotorGroupSpark(LEFT_PRIMARY_ID, new int[]{LEFT_FOLLOWER_ID}, MotorType.kBrushless);
        left.setInverted(true);
        MotorGroup right = new MotorGroupSpark(RIGHT_PRIMARY_ID, new int[]{RIGHT_FOLLOWER_ID}, MotorType.kBrushless);
        return new Drivetrain(left, right, TRACK_WIDTH, kS, kV, kA);
    }
    public String getChassisName() {
        return "Squishy";
    }
    public static final int LEFT_PRIMARY_ID = 1;
    public static final int RIGHT_PRIMARY_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;
}
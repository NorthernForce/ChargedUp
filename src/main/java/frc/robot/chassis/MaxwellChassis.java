// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import edu.wpi.first.math.util.Units;
import frc.lib.Motors.MotorGroup;
import frc.lib.Motors.MotorGroupTalonFX;
import frc.robot.subsystems.Drivetrain;


/** Chassis specific details about speedy. */
public class MaxwellChassis implements ChassisBase {
    public static final double TRACK_WIDTH = Units.inchesToMeters(22);
    public static final double kS = 0.071975;
    public static final double kV = 2.6116;
    public static final double kA = 0.27655;
    public MaxwellChassis() {}
    public Drivetrain getDrivetrain() {
        MotorGroup left = new MotorGroupTalonFX(LEFT_PRIMARY_ID, new int[]{LEFT_FOLLOWER_ID});
        MotorGroup right = new MotorGroupTalonFX(RIGHT_PRIMARY_ID, new int[]{RIGHT_FOLLOWER_ID});
        left.setInverted(true);
        return new Drivetrain(left, right, TRACK_WIDTH, kS, kV, kA);
    }
    public String getChassisName() {
        return "Maxwell";
    }
    public static final int LEFT_PRIMARY_ID = 1;
    public static final int RIGHT_PRIMARY_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;
}
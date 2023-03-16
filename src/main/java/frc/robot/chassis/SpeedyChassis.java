// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.lib.Motors.MotorGroup;
import frc.lib.Motors.MotorGroupTalonFX;
import frc.lib.cameras.PhotonCameraWrapper;
import frc.robot.subsystems.Drivetrain;


/** Chassis specific details about speedy. */
public class SpeedyChassis implements ChassisBase {
    public static final double TRACK_WIDTH = Units.inchesToMeters(24);
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public SpeedyChassis() {}
    public Drivetrain getDrivetrain() {
        MotorGroup left = new MotorGroupTalonFX(LEFT_PRIMARY_ID, new int[]{LEFT_FOLLOWER_ID});
        MotorGroup right = new MotorGroupTalonFX(RIGHT_PRIMARY_ID, new int[]{RIGHT_FOLLOWER_ID});
        right.setInverted(true);
        return new Drivetrain(left, right, TRACK_WIDTH, kS, kV, kA);
    }
    public String getChassisName() {
        return "Speedy";
    }
    public List<PhotonCameraWrapper> getPhotonCameras()
    {
        return null;
    };
    public static final int LEFT_PRIMARY_ID = 2;
    public static final int RIGHT_PRIMARY_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 3;
}
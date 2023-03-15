// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.Motors.MotorGroup;
import frc.lib.Motors.MotorGroupSpark;
import frc.robot.subsystems.Drivetrain;

/** Chassis specific details about squishy. */
public class SquishyChassis implements ChassisBase {
    public static final double TRACK_WIDTH = Units.inchesToMeters(24);
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public SquishyChassis() {}
    public Drivetrain getDrivetrain() {
        MotorGroup left = new MotorGroupSpark(MotorType.kBrushless, LEFT_PRIMARY_ID, LEFT_FOLLOWER_ID);
        MotorGroup right = new MotorGroupSpark(MotorType.kBrushless, RIGHT_PRIMARY_ID, RIGHT_FOLLOWER_ID);
        right.setInverted(true);
        return new Drivetrain(left, right, TRACK_WIDTH, kS, kV, kA);

    }
    public String getChassisName() {
        return "Squishy";
    }
    public static final int LEFT_PRIMARY_ID = 1;
    public static final int RIGHT_PRIMARY_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;
    public static class CameraLocations {
        public static final String LEFT_NAME = "Left";
        public static final double LEFT_X = Units.inchesToMeters(1.5);
        public static final double LEFT_Y = Units.inchesToMeters(6.75);
        public static final double LEFT_Z = Units.inchesToMeters(10);
        public static final Rotation3d LEFT_ROTATION = new Rotation3d(0, 0, Units.degreesToRadians(60));
        public static final Pose3d LEFT = new Pose3d(LEFT_X, LEFT_Y, LEFT_Z, LEFT_ROTATION);

        public static final String LEFT_FORWARD_NAME = "LeftForward";
        public static final double LEFT_FORWARD_X = Units.inchesToMeters(4);
        public static final double LEFT_FORWARD_Y = Units.inchesToMeters(6);
        public static final double LEFT_FORWARD_Z = Units.inchesToMeters(7);
        public static final Rotation3d LEFT_FORWARD_ROTATION = new Rotation3d(0, 0, 0);
        public static final Pose3d LEFT_FORWARD = new Pose3d(LEFT_FORWARD_X, LEFT_FORWARD_Y, LEFT_FORWARD_Z, LEFT_FORWARD_ROTATION);
    
        public static final String RIGHT_NAME = "Right";
        public static final double RIGHT_X = Units.inchesToMeters(1.5);
        public static final double RIGHT_Y = Units.inchesToMeters(-6.5);
        public static final double RIGHT_Z = Units.inchesToMeters(10.0871);
        public static final Rotation3d RIGHT_ROTATION = new Rotation3d(0, 0, Units.degreesToRadians(-60));
        public static final Pose3d RIGHT = new Pose3d(RIGHT_X, RIGHT_Y, RIGHT_Z, RIGHT_ROTATION);
    
        public static final String LIMELIGHT_NAME = "Limelight";
        public static final double LIMELIGHT_X = Units.inchesToMeters(5.5);
        public static final double LIMELIGHT_Y = Units.inchesToMeters(-5);
        public static final double LIMELIGHT_Z = Units.inchesToMeters(7);
        public static final Rotation3d LIMELIGHT_ROTATION = new Rotation3d(0, 0, 0);
        public static final Pose3d limeLight = new Pose3d(LIMELIGHT_X, LIMELIGHT_Y, LIMELIGHT_Z, LIMELIGHT_ROTATION);
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.Motors.MotorGroup;
import frc.lib.Motors.MotorGroupTalonFX;
import frc.lib.cameras.PhotonCameraWrapper;
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
    public List<PhotonCameraWrapper> getPhotonCameras()
    {
        return List.of(
            new PhotonCameraWrapper(CameraLocations.LEFT_NAME, CameraLocations.LEFT),
            new PhotonCameraWrapper(CameraLocations.LEFT_FORWARD_NAME, CameraLocations.LEFT_FORWARD),
            new PhotonCameraWrapper(CameraLocations.RIGHT_NAME, CameraLocations.RIGHT)
        );
    };
    public static final int LEFT_PRIMARY_ID = 1;
    public static final int RIGHT_PRIMARY_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    public static class CameraLocations {
        public static final String LEFT_NAME = "Left";
        public static final double LEFT_X = Units.inchesToMeters(5.75);
        public static final double LEFT_Y = Units.inchesToMeters(5.625);
        public static final double LEFT_Z = Units.inchesToMeters(19.125);
        public static final Rotation3d LEFT_ROTATION = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60));
        public static final Transform3d LEFT = new Transform3d(new Translation3d(LEFT_X, LEFT_Y, LEFT_Z), LEFT_ROTATION);

        public static final String LEFT_FORWARD_NAME = "LeftForward";
        public static final double LEFT_FORWARD_X = Units.inchesToMeters(4.5834);
        public static final double LEFT_FORWARD_Y = Units.inchesToMeters(4.72101);
        public static final double LEFT_FORWARD_Z = Units.inchesToMeters(23.625);
        public static final Rotation3d LEFT_FORWARD_ROTATION = new Rotation3d(Units.degreesToRadians(0), 0, 0);
        public static final Transform3d LEFT_FORWARD = new Transform3d(new Translation3d(LEFT_FORWARD_X, LEFT_FORWARD_Y, LEFT_FORWARD_Z), LEFT_FORWARD_ROTATION);
    
        public static final String RIGHT_NAME = "Right";
        public static final double RIGHT_X = Units.inchesToMeters(6);
        public static final double RIGHT_Y = Units.inchesToMeters(-5.625);
        public static final double RIGHT_Z = Units.inchesToMeters(18.625);
        public static final Rotation3d RIGHT_ROTATION = new Rotation3d(Units.degreesToRadians(0), 0, Units.degreesToRadians(-60));
        public static final Transform3d RIGHT = new Transform3d(new Translation3d(RIGHT_X, RIGHT_Y, RIGHT_Z), RIGHT_ROTATION);
     
        public static final String LIMELIGHT_NAME = "Limelight";
        public static final double LIMELIGHT_X = Units.inchesToMeters(5.33333);
        public static final double LIMELIGHT_Y = Units.inchesToMeters(4.5);
        public static final double LIMELIGHT_Z = Units.inchesToMeters(14.625);
        public static final Rotation3d LIMELIGHT_ROTATION = new Rotation3d(0, 0, 0);
        public static final Transform3d limeLight = new Transform3d(new Translation3d(LIMELIGHT_X, LIMELIGHT_Y, LIMELIGHT_Z), LIMELIGHT_ROTATION);
    }
}
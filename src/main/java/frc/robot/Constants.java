// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final int LEFT_PRIMARY_ID = 2;
    public static final int RIGHT_PRIMARY_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 3;
    public static final int UNITS_PER_REVOLUTION = 2048;
    public static final double GEAR_RATIO = 11.71;
    public static final double TRACK_WIDTH = Units.inchesToMeters(25);
    public static final double WHEEL_DIAMETER = Math.PI * Units.inchesToMeters(6);
    public static final double METERS_PER_REVOLUTION = WHEEL_DIAMETER / GEAR_RATIO;
    public static final double MAX_SPEED = 0; // TODO
    public static final double MAX_ACCELERATION = 0; // TODO
    public static final double kS = 0; // TODO
    public static final double kV = 0; // TODO
    public static final double kA = 0; // TODO
    public static final double DRIVE_PROPORTION = 0; // TODO
    public static final AprilTagFieldLayout APRILTAG_LAYOUT = new AprilTagFieldLayout(
        List.of(
            new AprilTag(1, new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(42.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, Math.toRadians(180))
            )),
            new AprilTag(2, new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(108.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, Math.toRadians(180))
            )),
            new AprilTag(3, new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(174.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, Math.toRadians(180))
            )),
            new AprilTag(4, new Pose3d(
                Units.inchesToMeters(636.96),
                Units.inchesToMeters(265.74),
                Units.inchesToMeters(27.38),
                new Rotation3d(0, 0, Math.toRadians(180))
            )),
            new AprilTag(5, new Pose3d(
                Units.inchesToMeters(14.25),
                Units.inchesToMeters(265.74),
                Units.inchesToMeters(27.38),
                new Rotation3d(0, 0, Math.toRadians(180))
            )),
            new AprilTag(6, new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(174.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, Math.toRadians(180))
            )),
            new AprilTag(7, new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(108.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, Math.toRadians(180))
            )),
            new AprilTag(8, new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(42.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, Math.toRadians(180))
            ))
        ), 
        16.4846, 
        8.1026
    );
    /** TODO */
    public static final Pose2d[] BLUE_POSES = new Pose2d[] {
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };
    /** TODO */
    public static final Pose2d[] RED_POSES = new Pose2d[] {
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };
}
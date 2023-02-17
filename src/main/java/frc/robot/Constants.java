package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.DynamicTransform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    /** General Constants */
    public static boolean ARM_ENABLED = false;
    public static boolean PCM_ENABLED = false;
    public static boolean DRIVETRAIN_ENABLED = false;
    public static boolean GRIPPER_ENABLED = false;
    public static boolean IMU_ENABLED = false;
    public static boolean LED_ENABLED = false;
    public static boolean NAVIGATION_ENABLED = false;
    public static boolean WRIST_ENABLED = false;
    public static boolean VISION_ENABLED = false;
    /** Arm Constants */
    public static double ARM_RAMP_RATE = 0.0;
    public static double ARM_EXTENDED_LENGTH = 0.0;
    public static int LEFT_PRIMARY_ID = 2;
    public static int RIGHT_PRIMARY_ID = 1;
    public static int LEFT_FOLLOWER_ID = 4;
    public static int RIGHT_FOLLOWER_ID = 3;
    public static double DRIVE_RAMP_RATE = 0.2;
    public static int UNITS_PER_REVOLUTION = 2048;
    public static double GEAR_RATIO = 15.5;
    public static double TRACK_WIDTH = Units.inchesToMeters(24);
    public static double WHEEL_DIAMETER = Math.PI * Units.inchesToMeters(6);
    public static double METERS_PER_REVOLUTION = WHEEL_DIAMETER / GEAR_RATIO;
    public static double MAX_SPEED = 0; // TODO
    public static double MAX_ACCELERATION = 0; // TODO
    public static double kS = 0; // TODO
    public static double kV = 0; // TODO
    public static double kA = 0; // TODO
    public static double LEFT_DRIVE_PROPORTION = 0; // TODO
    public static double RIGHT_DRIVE_PROPORTION = 0; // TODO
    public static double FAST_SPEED_FORWARD = 1.0;
    public static double FAST_SPEED_ROTATION = 0.75;
    public static double SLOW_SPEED_FORWARD = 0.25;
    public static double SLOW_SPEED_ROTATION = 0.4;
    /** Navigation Constants */
    public static String NAVIGATION_CAMERA_NAME = "apriltagCamera";
    public static Transform3d NAVIGATION_CAMERA_TRANSFORM = new DynamicTransform3d();
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
    /** IMU Constants */
    public static boolean ROLL_PITCH_SWAPPED = false;
    /** LED Constants */
    public static int LED_NUM_LEDS = 40;
    public static int LED_PORT = 0;
    /** Vision Constants */
    public static String VISION_CAMERA_NAME = "visionCamera";
    /** Arm Constants */
    public static int ARM_LEFT_MOTOR = 5;
    public static int ARM_RIGHT_MOTOR = 6;
    public static double ARM_RETRACTED_LENGTH = 0.0; // TODO
    public static Translation3d ARM_ORIGIN = null; // TODO
    /** Compressor Constants */
    public static int COMPRESSOR_ID = 7;
    /** Solenoids */
    public static int MOTOR_SOLENOID_FORWARD = 2;
    public static int MOTOR_SOLENOID_REVERSE = 3;
    public static int MOTOR_SOLENOID_ID = 1;
    /** Wrist Constants */
    public static int WRIST_MOTOR_ID = 10;
    public static double WRIST_PROPORTION = 0.0; // TODO
    public static double WRIST_GEAT_RATIO = 0.0; // TODO
}
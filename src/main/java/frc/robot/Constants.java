package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    /** Drive Constants */
    public static final int LEFT_PRIMARY_ID = 2;
    public static final int RIGHT_PRIMARY_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 3;
    public static final double DRIVE_RAMP_RATE = 0.2;
    public static final int UNITS_PER_REVOLUTION = 2048;
    public static final double GEAR_RATIO = 15.5;
    public static final double TRACK_WIDTH = Units.inchesToMeters(24);
    public static final double WHEEL_DIAMETER = Math.PI * Units.inchesToMeters(6);
    public static final double METERS_PER_REVOLUTION = WHEEL_DIAMETER / GEAR_RATIO;
    public static final double MAX_SPEED = 0; // TODO
    public static final double MAX_ACCELERATION = 0; // TODO
    public static final double kS = 0; // TODO
    public static final double kV = 0; // TODO
    public static final double kA = 0; // TODO
    public static final double LEFT_DRIVE_PROPORTION = 0; // TODO
    public static final double RIGHT_DRIVE_PROPORTION = 0; // TODO
    public static final double FAST_SPEED_FORWARD = 1.0;
    public static final double FAST_SPEED_ROTATION = 0.75;
    public static final double SLOW_SPEED_FORWARD = 0.25;
    public static final double SLOW_SPEED_ROTATION = 0.4;
    /** Navigation Constants */
    public static final String NAVIGATION_CAMERA_NAME = "apriltagCamera";
    public static final Transform3d NAVIGATION_CAMERA_TRANSFORM = new DynamicTransform3d();
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
    /** General Constants */
    public static final boolean ARM_ENABLED = false;
    public static final boolean COMPRESSOR_ENABLED = false;
    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean GRIPPER_ENABLED = false;
    public static final boolean IMU_ENABLED = true;
    public static final boolean LED_ENABLED = true;
    public static final boolean NAVIGATION_ENABLED = false;
    public static final boolean WRIST_ENABLED = false;
    public static final boolean VISION_ENABLED = false;
    /** IMU Constants */
    public static final boolean ROLL_PITCH_SWAPPED = false;
    /** LED Constants */
    public static final int LED_NUM_LEDS = 40;
    public static final int LED_PORT = 0;
    /** Vision Constants */
    public static final String VISION_CAMERA_NAME = "visionCamera";
    public static final double VISION_CAMERA_HEIGHT = Units.inchesToMeters(4);
    public static final Rotation2d VISION_CAMERA_PITCH = Rotation2d.fromDegrees(0);
    /** Arm Constants */
    public static final int ARM_MOTOR_ID = 5;
    public static final int ARM_ROTATE_CANCODER_ID = 13;
    public static final double ARM_RAMP_RATE = 0.2;
    public static final double ARM_EXTENDED_LENGTH = Units.inchesToMeters(40.0);
    public static final double ARM_RETRACTED_LENGTH = Units.inchesToMeters(26.0);
    public static final Translation3d ARM_ORIGIN = new Translation3d(
        Units.inchesToMeters(3),
        Units.inchesToMeters(0),
        Units.inchesToMeters(26.5)
    );
    public static final double ARM_PROPORTION = 0.0; // TODO
    /** Gripper Constants */
    public static final int GRIPPER_MOTOR_ID = 9;
    public static final double GRIPPER_CONE_INTAKE_SPEED = 1.0; // TODO
    public static final double GRIPPER_CONE_OUTTAKE_SPEED = -1.0; // TODO
    /** Compressor Constants */
    public static final int COMPRESSOR_ID = 20;
    /** Solenoids */
    public static final int TELESCOPE_SOLENOID_FORWARD = 0;
    public static final int TELESCOPE_SOLENOID_REVERSE = 1;
    public static final int TELESCOPE_SOLENOID_ID = 0;
    public static final int MOTOR_SOLENOID_FORWARD = 2;
    public static final int MOTOR_SOLENOID_REVERSE = 3;
    public static final int MOTOR_SOLENOID_ID = 1;
    /** Wrist Constants */
    public static final int WRIST_MOTOR_ID = 10;
    public static final double WRIST_PROPORTION = 0.0; // TODO
    public static final double WRIST_GEAT_RATIO = 0.0; // TODO
}

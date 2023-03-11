package frc.robot;

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
    public static class DrivetrainConstants
    {
        public static final double RAMP_RATE = 0.2;
        public static final int UNITS_PER_REVOLUTION = 2048;
        public static final double GEAR_RATIO = 11.37777778;
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        public static final double WHEEL_DIAMETER = Math.PI * Units.inchesToMeters(6);
        public static final double METERS_PER_REVOLUTION = WHEEL_DIAMETER / GEAR_RATIO;
        public static final double MAX_SPEED = 0.1; // TODO
        public static final double MAX_ACCELERATION = 0.1; // TODO
        public static final double kS = 0.071975;
        public static final double kV = 2.6116;
        public static final double kA = 0.27655;
        public static final double LEFT_DRIVE_PROPORTION = 0.50935;
        public static final double RIGHT_DRIVE_PROPORTION = 0.50935;
        public static final double FAST_SPEED_FORWARD = 1.0;
        public static final double FAST_SPEED_ROTATION = 0.75;
        public static final double SLOW_SPEED_FORWARD = 0.25;
        public static final double SLOW_SPEED_ROTATION = 0.4;
    }
    /** Navigation Constants */
    public static class NavigationConstants
    {
        public static final String CAMERA1_NAME = "apriltagCamera1";
        public static final String CAMERA2_NAME = "apriltagCamera2";
        public static final String CAMERA3_NAME = "apriltagCamera3";
        /** ROBOT CENTER TO Camera */
        public static final Transform3d CAMERA1_TRANSFORM = new DynamicTransform3d(
            new Translation3d(
                Units.inchesToMeters(11), /** X */
                Units.inchesToMeters(3), /** Y */
                Units.inchesToMeters(12.5) /** A */
            ),
            new Rotation3d(
                0, /** Roll */
                0, /** Pitch */
                Math.toRadians(12) /** Yaw */
            )
        );
        /** ROBOT CENTER TO Camera */
        public static final Transform3d CAMERA2_TRANSFORM = new DynamicTransform3d(
            new Translation3d(
                Units.inchesToMeters(11), /** X */
                Units.inchesToMeters(3), /** Y */
                Units.inchesToMeters(12.5) /** A */
            ),
            new Rotation3d(
                0, /** Roll */
                0, /** Pitch */
                Math.toRadians(12) /** Yaw */
            )
        );
        /** ROBOT CENTER TO Camera */
        public static final Transform3d CAMERA3_TRANSFORM = new DynamicTransform3d(
            new Translation3d(
                Units.inchesToMeters(11), /** X */
                Units.inchesToMeters(3), /** Y */
                Units.inchesToMeters(12.5) /** A */
            ),
            new Rotation3d(
                0, /** Roll */
                0, /** Pitch */
                Math.toRadians(12) /** Yaw */
            )
        );
    }
    /** General Constants */
    /** IMU Constants */
    public static class IMUConstants
    {
        public static final boolean ROLL_PITCH_SWAPPED = false;
    }
    /** LED Constants */
    public static class LEDConstants
    {
        public static final int NUM_LEDS = 40;
        public static final int PORT = 0;
    }
    /** Vision Constants */
    public static class VisionConstants
    {
        public static final String CAMERA_NAME = "visionCamera";
        public static final double CAMERA_HEIGHT = Units.inchesToMeters(4);
        public static final Rotation2d CAMERA_PITCH = Rotation2d.fromDegrees(0);
        public static final Transform3d TRANSFORM3D = new DynamicTransform3d(
            new Translation3d(
                0, // TODO
                0, // TODO
                Units.inchesToMeters(CAMERA_HEIGHT)
            ),
            new Rotation3d(
                0,
                CAMERA_PITCH.getRadians(),
                0
            )
        );
    }
    /** Arm Constants */
    public static class ArmConstants
    {
        public static final int PRIMARY_MOTOR_ID = 5;
        public static final int FOLLOWER_MOTOR_ID = 6;
        public static final int CANCODER_ID = 13;
        public static final double RAMP_RATE = 0.2;
        public static final double EXTENDED_LENGTH = Units.inchesToMeters(40.0);
        public static final double RETRACTED_LENGTH = Units.inchesToMeters(26.0);
        public static final Translation3d ORIGIN = new Translation3d(
            Units.inchesToMeters(3),
            Units.inchesToMeters(0),
            Units.inchesToMeters(26.5)
        );
        public static final double kP = 0.0; // TODO
        public static final double kF = 0.0; // TODO
        public static final double kI = 0.0; // TODO
        public static final double kD = 0.0; // TODO
        public static final double kFF = 0.0; // TODO
        public static final double ANGLE_TOLERANCE = 1.0; // TODO
        public static final double CANCODER_OFFSET = 0.25;  // Changes from height line = 0 degrees to horizon line = 0 defgrees
    }
    /** Gripper Constants */
    public static class GripperConstants
    {
        public static final int MOTOR_ID = 9;
        public static final double CONE_INTAKE_SPEED = -1.0; // TODO
        public static final double CONE_OUTTAKE_SPEED = 1.0; // TODO
    }
    /** Compressor Constants */
    public static class CompressorConstants
    {
        public static final int COMPRESSOR_ID = 20;
        /** Solenoids */
        public static final int TELESCOPE_SOLENOID_ID = 0;
    }
    /** Wrist Constants */
    public static class WristConstants
    {
        public static final int MOTOR_ID = 10;
        public static final double GEAT_RATIO = 0.0; // TODO
        // Static gain
        public static final double kF = 0.0; // TODO
        // Proportion
        public static final double kP = 0.0; // TODO
        // Integral
        public static final double kI = 0.0; // TODO
        // Derivative
        public static final double kD = 0.0; // TODO
        // Feedforward or gravity Gain
        public static final double kFF = 0.0; // TODO
    }
}

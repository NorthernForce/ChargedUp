package frc.robot;

import edu.wpi.first.math.Pair;
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
    /**
     * Utility function to calculate the arm position for any given target height, arm constants, and wrist position.
     * Units does not matter as long as consistent. That said, this programming repository prefers to operate in the metric system in code.
     * @param armHeight The height of the arm fulcrum off of the same surface as the target, ideally the floor
     * @param armLength The length of the arm from fulcrum to wrist fulcrum.
     * @param wristFulcrumToEnd The distance from the wrist fulcrum to the place that the gripper or rollers are.
     * @param wristAngle Angle that the wrist will be at for the purposes of the arm calculation. Off of the floor.
     * @param targetHeight The height of the target off of the same surface as the arm height, ideally the floor.
     * @param armToCenter the amount of units that the arm is in front of the center of the robot
     * @return a pair of distance to the target, as well as the ideal angle
     */
    public static Pair<Double, Rotation2d> calculateArmAngleAndDistance(double armHeight, double armLength, double wristFulcrumToEnd, Rotation2d wristAngle, double targetHeight, double armToCenter)
    {
        double heightDiff = targetHeight - armHeight - wristAngle.getSin() * wristFulcrumToEnd;
        Rotation2d armAngle = Rotation2d.fromRadians(Math.asin(heightDiff / armLength));
        double targetDistance = armAngle.getCos() * armLength + wristAngle.getCos() * wristFulcrumToEnd + armToCenter;
        return new Pair<Double,Rotation2d>(targetDistance, armAngle);
    }
    /** Drive Constants */
    public static class DrivetrainConstants
    {
        public static final double RAMP_RATE = 0.2;
        public static final int UNITS_PER_REVOLUTION = 2048;
        public static final double GEAR_RATIO = 11.37777778;
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        public static final double WHEEL_DIAMETER = Math.PI * Units.inchesToMeters(6);
        public static final double METERS_PER_REVOLUTION = WHEEL_DIAMETER / GEAR_RATIO;
        public static final double MAX_SPEED = 0.4; // TODO
        public static final double MAX_ACCELERATION = 0.4; // TODO
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
    /** Angle and distance pairs*/
    public static class AnglesAndDistances {
        //Intake Positions
        public static final Pair<Double, Rotation2d> FLOOR_CONE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CONE, 
            WristConstants.FLOOR_CONE_PICKUP_ANGLE, 
            PiceConstants.CONE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );
        
        public static final Pair<Double, Rotation2d> FLOOR_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.FLOOR_CUBE_PICKUP_ANGLE, 
            PiceConstants.CUBE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );

        //Outtake positions
        public static final Pair<Double, Rotation2d> HIGH_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.EXTENDED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.HIGH_CUBE_PLACEMENT_ANGLE, 
            FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[1].getZ() + PiceConstants.CUBE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );
        public static final Pair<Double, Rotation2d> MEDIUM_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.MID_CUBE_PLACEMENT_ANGLE,
            FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[0].getZ() + PiceConstants.CUBE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );
        public static final Pair<Double, Rotation2d> LOW_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.FLOOR_CUBE_PLACEMENT_ANGLE,
            PiceConstants.CUBE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );
        
        public static final Pair<Double, Rotation2d> HIGH_CONE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.EXTENDED_LENGTH, 
            GripperConstants.FULCRUM_TO_CONE, 
            WristConstants.HIGH_CONE_PLACEMENT_ANGLE, 
            FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS[1].getZ() + PiceConstants.CONE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );
        public static final Pair<Double, Rotation2d> MEDIUM_CONE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CONE, 
            WristConstants.MID_CONE_PLACEMENT_ANGLE, 
            FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS[0].getZ() + PiceConstants.CONE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );
        public static final Pair<Double, Rotation2d> LOW_CONE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.FLOOR_CONE_PLACEMENT_ANGLE,
            PiceConstants.CONE_HEIGHT, 
            ArmConstants.ORIGIN.getX()
        );
    }
    /** Pice dimensions constants*/
    public static class PiceConstants {
        public static final double CONE_HEIGHT = Units.inchesToMeters(13); //TODO
        public static final double CUBE_HEIGHT = Units.inchesToMeters(9.5); //TODO
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
        public static final double CAMERA_HEIGHT = Units.inchesToMeters(4); // TODO
        public static final Rotation2d CAMERA_PITCH = Rotation2d.fromDegrees(0); // TODO
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
        public static final double kP = 2.0; // TODO
        public static final double kF = 0.0; // TODO
        public static final double kI = 0.0; // TODO
        public static final double kD = 0.0; // TODO
        public static final double kFF = 0.0; // TODO
        public static final double ANGLE_TOLERANCE = 1.0; // TODO
        public static final double CANCODER_OFFSET = 0.25;  // Changes from height line = 0 degrees to horizon line = 0 defgrees
        public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(-40);
        public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(197);
    }
    /** Gripper Constants */
    public static class GripperConstants
    {
        public static final int MOTOR_ID = 9;
        public static final double CONE_INTAKE_SPEED = -1.0; // TODO
        public static final double CONE_OUTTAKE_SPEED = 1.0; // TODO
        public static final double FULCRUM_TO_CONE = Units.inchesToMeters(14.5); // 14.5" cone
        public static final double FULCRUM_TO_CUBE = Units.inchesToMeters(9); // 9" cube
        public static final double INTAKE_TIME = 0.5;
        public static final double OUTTAKE_TIME = 0.5;
        public static final double FOLCRUM_TO_CONE = Units.inchesToMeters(14.5); // 14.5" cone
        public static final double FOLCRUM_TO_CUBE = Units.inchesToMeters(9); // 9" cube
        
    }
    /** Compressor Constants */
    public static class CompressorConstants
    {
        public static final int COMPRESSOR_ID = 20;
        /** Solenoids */
        public static final int TELESCOPE_SOLENOID_ID = 8;
    }
    /** Wrist Constants */
    public static class WristConstants
    {
        public static final int MOTOR_ID = 10;
        public static final int CANCODER_ID = 12;
        public static final double GEAT_RATIO = 0.0; // TODO
        // Static gain
        public static final double kF = 0.0; // TODO
        // Proportion
        public static final double kP = 2.0; // TODO
        // Integral
        public static final double kI = 0.0; // TODO
        // Derivative
        public static final double kD = 0.0; // TODO
        // Feedforward or gravity Gain
        public static final double kFF = 0.0; // TODO
        public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(-30);
        public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(170);

        // these angles are field relative
        // pickup angle
        public static final Rotation2d FLOOR_CONE_PICKUP_ANGLE = Rotation2d.fromDegrees(0); //TODO
        public static final Rotation2d FLOOR_CUBE_PICKUP_ANGLE = Rotation2d.fromDegrees(0); //TODO
        
        // placement angles
        public static final Rotation2d HIGH_CUBE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0); //TODO
        public static final Rotation2d MID_CUBE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0); //TODO
        public static final Rotation2d FLOOR_CUBE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0); //TODO
        
        public static final Rotation2d HIGH_CONE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0); //TODO
        public static final Rotation2d MID_CONE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0); //TODO
        public static final Rotation2d FLOOR_CONE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0); //TODO

        //overshot and dropdowns
        public static final Rotation2d CUBE_OVERSHOOT = Rotation2d.fromDegrees(20); // TODO
        public static final Rotation2d CONE_OVERSHOOT = Rotation2d.fromDegrees(20); // TODO
        public static final Rotation2d CUBE_DROPDOWN = Rotation2d.fromDegrees(10); // TODO
        public static final Rotation2d CONE_DROPDOWN = Rotation2d.fromDegrees(10); // TODO
    }   
}

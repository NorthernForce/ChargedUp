package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
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
    public static Pair<Double, Rotation2d> calculateArmAngleAndDistance(double armHeight, double armLength, double wristFulcrumToEnd, Rotation2d wristAngle, double targetHeight, double armToCenter, Rotation2d armOvershoot)
    {
        double heightDiff = targetHeight - armHeight - wristAngle.getSin() * wristFulcrumToEnd;
        Rotation2d armAngle = Rotation2d.fromRadians(Math.asin(heightDiff / armLength));
        double targetDistance = armAngle.getCos() * armLength + wristAngle.getCos() * wristFulcrumToEnd + armToCenter;
        return new Pair<Double,Rotation2d>(targetDistance, armAngle.plus(armOvershoot));
    }
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
    public static Pair<Double, Rotation2d> calculateArmAngleAndDistance(double armHeight, double armLength, double wristFulcrumToEnd, Rotation2d wristAngle, double targetHeight, double armToCenter, Rotation2d armOvershoot, boolean reversed)
    {
        if (!reversed)
        {
            double heightDiff = targetHeight - armHeight - wristAngle.getSin() * wristFulcrumToEnd;
            Rotation2d armAngle = Rotation2d.fromRadians(MathUtil.inputModulus(Math.asin(heightDiff / armLength), Math.toRadians(-90), Math.toRadians(270)));
            double targetDistance = armAngle.getCos() * armLength + wristAngle.getCos() * wristFulcrumToEnd + armToCenter;
            return new Pair<Double,Rotation2d>(targetDistance, armAngle.plus(armOvershoot));
        }
        else
        {
            double heightDiff = targetHeight - armHeight - wristAngle.getSin() * wristFulcrumToEnd;
            Rotation2d armAngle = Rotation2d.fromRadians(MathUtil.inputModulus(Math.asin(heightDiff / armLength), Math.toRadians(-90), Math.toRadians(270)));
            double targetDistance = armAngle.getCos() * armLength + wristAngle.getCos() * wristFulcrumToEnd + armToCenter;
            return new Pair<Double,Rotation2d>(targetDistance, Rotation2d.fromRotations(0.5).minus(armAngle).plus(armOvershoot));
        }
    }
    /** Drive Constants */
    public static class DrivetrainConstants
    {
        public static final double RAMP_RATE = 0.2;
        public static final int UNITS_PER_REVOLUTION = 2048;
        public static final double GEAR_RATIO = 11.37777778;
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(6);
        public static final double METERS_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
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
        public static final double LEFT_MIRCO_ADJUST = 0.5;
        public static final double RIGHT_MIRCO_ADJUST = -0.5;
        public static final Rotation2d MICRO_ADJUST_DEGREES = Rotation2d.fromDegrees(2);
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
        public static final double kP = 1.2; // TODO
        public static final double kMaxCountsPer100MS = 117.16;
        public static final double kF = (0.85 * 1023) / 117.16;
        public static final double kInitialVelocity = kMaxCountsPer100MS * 0.6;
        public static final double kInitialAcceleration = kInitialVelocity * 5;
        public static final double kI = 0.0; // TODO
        public static final double kD = 0.0; // TODO
        public static final double kFF = 0.0; // TODO
        public static final double kIntegralZone = 0.0; // TODO
        public static final double ANGLE_TOLERANCE = 5; // TODO
        public static final double CANCODER_OFFSET = 0.25;  // Changes from height line = 0 degrees to horizon line = 0 defgrees
        public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(-45);
        public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(197);
        public static final Rotation2d NORTH_ANGLE = Rotation2d.fromDegrees(90),
            SOUTH_ANGLE = Rotation2d.fromDegrees(-43), // TODO
            EAST_ANGLE = Rotation2d.fromDegrees(160), // TODO
            WEST_ANGLE = Rotation2d.fromDegrees(0); // TODO
    }
    /** Gripper Constants */
    public static class GripperConstants
    {
        public static final int MOTOR_ID = 9;
        public static final double CONE_INTAKE_SPEED = -1.0; // TODO
        public static final double CONE_OUTTAKE_SPEED = 1.0; // TODO
        public static final double FULCRUM_TO_CUBE = Units.inchesToMeters(9);
        public static final double FULCRUM_TO_CONE = Units.inchesToMeters(14.5);
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
        public static final double kF = 0.0; // TODOP
        // Proportion
        public static final double kP = 0.1; // TODO
        // Integral
        public static final double kI = 0.0; // TODO
        // Derivative
        public static final double kD = 2.0; // TODO
        // Feedforward or gravity Gain
        public static final double kFF = 0.20; // TODO
        public static final double kMaxAccel = 0.0; // TODO
        public static final double kMaxVelocity = 0.0; // TODO
        public static final double kMinOutputVelocity = 0.0; // TODO
        public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(95);
        public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(265);
        public static final Rotation2d HIGH_CUBE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d LOW_CUBE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MID_CUBE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d LOW_CONE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MID_CONE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d HIGH_CONE_PLACEMENT_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d NORTH_ANGLE = Rotation2d.fromDegrees(0),
            SOUTH_ANGLE = Rotation2d.fromDegrees(66), // TODO
            EAST_ANGLE = Rotation2d.fromDegrees(60), // TODO
            WEST_ANGLE = Rotation2d.fromDegrees(40); // TODO
        public static final Rotation2d BACKWARD_PICKUP_ANGLE = Rotation2d.fromDegrees(160);
    }
    public static class AnglesAndDistances
    {
        //Outtake positions
        public static final Pair<Double, Rotation2d> HIGH_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.EXTENDED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.HIGH_CUBE_PLACEMENT_ANGLE, 
            FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[1].getZ() + PiceConstants.CUBE_HEIGHT, 
            ArmConstants.ORIGIN.getX(),
            Rotation2d.fromDegrees(14)
        );
        public static final Pair<Double, Rotation2d> MEDIUM_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.MID_CUBE_PLACEMENT_ANGLE,
            FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[0].getZ() + PiceConstants.CUBE_HEIGHT, 
            ArmConstants.ORIGIN.getX(),
            Rotation2d.fromDegrees(19)
        );
        public static final Pair<Double, Rotation2d> LOW_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.LOW_CUBE_PLACEMENT_ANGLE,
            PiceConstants.CUBE_HEIGHT, 
            ArmConstants.ORIGIN.getX(),
            Rotation2d.fromDegrees(19)
        );

        public static final Pair<Double, Rotation2d> HIGH_CONE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.EXTENDED_LENGTH, 
            GripperConstants.FULCRUM_TO_CONE, 
            WristConstants.HIGH_CONE_PLACEMENT_ANGLE, 
            FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS[1].getZ() + PiceConstants.CONE_HEIGHT, 
            ArmConstants.ORIGIN.getX(),
            Rotation2d.fromDegrees(19)
        );
        public static final Pair<Double, Rotation2d> MEDIUM_CONE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CONE, 
            WristConstants.MID_CONE_PLACEMENT_ANGLE, 
            FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS[0].getZ() + PiceConstants.CONE_HEIGHT, 
            ArmConstants.ORIGIN.getX(),
            Rotation2d.fromDegrees(19)
        );
        public static final Pair<Double, Rotation2d> LOW_CONE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(), 
            ArmConstants.RETRACTED_LENGTH, 
            GripperConstants.FULCRUM_TO_CUBE, 
            WristConstants.LOW_CONE_PLACEMENT_ANGLE,
            PiceConstants.CONE_HEIGHT, 
            ArmConstants.ORIGIN.getX(),
            Rotation2d.fromDegrees(19)
        );
        public static final Pair<Double, Rotation2d> BACKWARD_FLOOR_CUBE = calculateArmAngleAndDistance(
            ArmConstants.ORIGIN.getZ(),
            ArmConstants.EXTENDED_LENGTH,
            GripperConstants.FULCRUM_TO_CUBE,
            WristConstants.BACKWARD_PICKUP_ANGLE,
            PiceConstants.CUBE_HEIGHT,
            ArmConstants.ORIGIN.getX(),
            Rotation2d.fromDegrees(360 -15),
            true
        );
    }
    /** Pice dimensions constants*/
    public static class PiceConstants {
        public static final double CONE_HEIGHT = Units.inchesToMeters(13); //TODO
        public static final double CUBE_HEIGHT = Units.inchesToMeters(9.5); //TODO
    }
    public static enum PathSource
    {
        PATHWEAVER("paths/"),
        PATHPLANNER("pathplanner/generatedJSON/");
        private final String directory;
        private PathSource(String directory)
        {
            this.directory = directory;
        }
        public Trajectory load(String pathName) throws IOException
        {
            return TrajectoryUtil.fromPathweaverJson(
              Filesystem.getDeployDirectory().toPath().resolve(directory + pathName + ".wpilib.json")
            );
        }
    };
    public static enum Path
    {
        // Pathweaver paths
        BLUE_LEFT_TO_PIECE_LEFT("Blue3ToPiece4", PathSource.PATHWEAVER),
        PIECE_LEFT_TO_BLUE_LEFT("Piece4ToBlue3", PathSource.PATHWEAVER),
        BLUE_LEFT_TO_PIECE_LEFT_CENTER("Blue3ToPiece3", PathSource.PATHWEAVER),
        PIECE_LEFT_CENTER_TO_BLUE_LEFT("Piece3ToBlue3", PathSource.PATHWEAVER),
        BLUE_RIGHT_TO_PIECE_RIGHT("Blue1ToPiece1", PathSource.PATHWEAVER),
        PIECE_RIGHT_TO_BLUE_RIGHT("Piece1ToBlue1", PathSource.PATHWEAVER),
        BLUE_RIGHT_TO_PIECE_RIGHT_CENTER("Blue3ToPiece3", PathSource.PATHWEAVER),
        PIECE_RIGHT_CENTER_TO_BLUE_RIGHT("Piece3ToBlue3", PathSource.PATHWEAVER),
        RED_LEFT_TO_PIECE_LEFT("Red1ToPiece1", PathSource.PATHWEAVER),
        PIECE_LEFT_TO_RED_LEFT("Piece1ToRed1", PathSource.PATHWEAVER),
        RED_LEFT_TO_PIECE_LEFT_CENTER("Red1ToPiece2", PathSource.PATHWEAVER),
        PIECE_LEFT_CENTER_TO_RED_LEFT("Piece2ToRed1", PathSource.PATHWEAVER),
        RED_RIGHT_TO_PIECE_RIGHT("Red3ToPiece4", PathSource.PATHWEAVER),
        PIECE_RIGHT_TO_RED_RIGHT("Piece4ToRed3", PathSource.PATHWEAVER),
        RED_RIGHT_TO_PIECE_RIGHT_CENTER("Red3ToPiece3", PathSource.PATHWEAVER),
        PIECE_RIGHT_CENTER_TO_RED_RIGHT("Red3ToBlue3", PathSource.PATHWEAVER),
        // Path Planner Paths
        BACKWARD_BLUE_LEFT_TO_PIECE_LEFT("BlueLeftPieceLeft", PathSource.PATHPLANNER),
        FORWARD_PIECE_LEFT_TO_BLUE_LEFT("PieceLeftBlueLeft", PathSource.PATHPLANNER),
        BACKWARD_BLUE_RIGHT_TO_PIECE_RIGHT("BlueRightPieceRight", PathSource.PATHPLANNER),
        FORWARD_PIECE_RIGHT_TO_BLUE_RIGHT("PieceRightBlueRight", PathSource.PATHPLANNER),
        BACKWARD_RED_LEFT_TO_PIECE_LEFT("RedLeftPieceLeft", PathSource.PATHPLANNER),
        FORWARD_PIECE_LEFT_TO_RED_LEFT("PieceLeftRedLeft", PathSource.PATHPLANNER),
        BACKWARD_RED_RIGHT_TO_PIECE_RIGHT("RedRightPieceRight", PathSource.PATHPLANNER),
        FORWARD_PIECE_RIGHT_TO_RED_RIGHT("PieceRightRedRight", PathSource.PATHPLANNER);
        private String pathName;
        private final PathSource pathSource;
        private Path(String pathName, PathSource source)
        {
            this.pathName = pathName;
            this.pathSource = source;
        }
        public Trajectory load() throws IOException
        {
            return pathSource.load(pathName);
        }
    };
}

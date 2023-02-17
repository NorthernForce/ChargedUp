package frc.robot;

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
    public static boolean VISION_ENABLED = false;
    public static boolean ROLL_PITCH_SWAPPED = false;
    /** Arm Constants */
    public static double ARM_RAMP_RATE = 0.0;
    public static double ARM_EXTENDED_LENGTH = 0.0;
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    // Targeting constants
    public static final String CAMERA_ID = "webcam";
    public static final double CAMERA_HEIGHT_METERS = 0.22;
    public static final double TARGET_HEIGHT_METERS = 0.77;
    public static final double CAMERA_PITCH_RADIANS = 0;
    //drive constants
    public static final int LEFT_PRIMARY_ID = 2;
    public static final int RIGHT_PRIMARY_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 3;

    public static final double DRIVE_RAMP_RATE = 0.2;

    public static final double ROTATION_SPEED_PROPORTION = 0.75;
    public static final double SPEED_PROPORTION = 1;

    public static final int kUnitsPerRevolution = 2048;

    //shooter constants
    public static final int SHOOTER_FLYWHEEL_ID = 5;
    public static final int SHOOTER_HOOD_ROTATOR_ID = 91; //FIXME fix can id's
    public static final int SHOOTER_HOOD_POWER_ID = 90;

    public static final double SHOOTER_RAMP_RATE = 1;
    
    //climber constants
    public static final int LEFT_CLIMBER_ID = 8;
    public static final int RIGHT_CLIMBER_ID = 7;

    public static final int PCM_CAN_ID = 15;
    public static final int CLIMBER_SOLENOID_ID = 7;
    public static final int ARM_SOLENOID_ID = 9;

    public static final int BOTTOM_OPTICAL_ID = 0;
    public static final int MIDDLE_OPTICAL_ID = 2;
    public static final int TOP_OPTICAL_ID = 1;

    //intake constants
    public static final int INTAKE_MOTOR_ID = 93; // FIXME fix intake motor id
    public static final int INTAKE_DIRECTION = 1; //FIXME set to make the buttons do the right thing
    public static final double INTAKE_SPEED_PROPORTION = 0.2;
    public static final int INTAKE_SOLENOID_ID = 94; // FIXME fix intake solenoid id
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.variants.DrivetrainSquishy;
import frc.robot.util.DynamicTransform3d;

/** Chassis specific details about squishy. */
public class SquishyChassis implements ChassisBase {
    private static final Map<String, Double> DOUBLE_CONSTANTS = Map.ofEntries(
        Map.entry("TRACK_WIDTH", Units.inchesToMeters(24)),
        Map.entry("METERS_PER_REVOLUTION", Math.PI * Units.inchesToMeters(6) / 11.71),
        Map.entry("MAX_SPEED", 0.0), // TODO
        Map.entry("MAX_ACCELERATION", 0.0), // TODO
        Map.entry("kS", 0.0), // TODO
        Map.entry("kV", 0.0), // TODO
        Map.entry("kA", 0.0), // TODO
        Map.entry("LEFT_DRIVE_PROPORTION", 0.0), // TODO
        Map.entry("RIGHT_DRIVE_PROPORTION", 0.0), // TODO
        Map.entry("FAST_SPEED_FORWARD", 1.0),
        Map.entry("FAST_SPEED_ROTATION", 0.75),
        Map.entry("SLOW_SPEED_FORWARD", 0.25),
        Map.entry("SLOW_SPEED_ROTATION", 0.4),
        Map.entry("ARM_RAMP_RATE", 0.2),
        Map.entry("ARM_EXTENDED_LENGTH", 0.0), // TODO
        Map.entry("ARM_RETRACTED_LENGTH", 0.0) // TODO
    );
    private static final Map<String, String> STRING_CONSTANTS = Map.of(
        "NAVIGATION_CAMERA_NAME", "apriltagCamera",
        "VISION_CAMERA_NAME", "visionCamera"
    );
    private static final Map<String, Integer> INTEGER_CONSTANTS = Map.ofEntries(
        Map.entry("LEFT_PRIMARY_ID", 1),
        Map.entry("RIGHT_PRIMARY_ID", 2),
        Map.entry("LEFT_FOLLOWER_ID", 3),
        Map.entry("RIGHT_FOLLOWER_ID", 4),
        Map.entry("NUM_ACTIVE_LEDS", 40),
        Map.entry("LED_PORT", 0),
        Map.entry("ARM_LEFT_MOTOR_ID", 5),
        Map.entry("ARM_RIGHT_MOTOR_ID", 6),
        Map.entry("ARM_POTENTIOMETER_ID", 0),
        Map.entry("COMPRESSOR_ID", 7),
        Map.entry("MOTOR_SOLENOID_FORWARD", 2),
        Map.entry("MOTOR_SOLENOID_REVERSE", 3),
        Map.entry("MOTOR_SOLENOID_ID", 1)
    );
    private static final Map<String, Boolean> BOOLEAN_CONSTANTS = Map.of(
        "ARM_ENABLED", false,
        "PCM_ENABLED", false,
        "DRIVETRAIN_ENABLED", true,
        "GRIPPER_ENABLED", false,
        "IMU_ENABLED", true,
        "LED_ENABLED", false,
        "NAVIGATION_ENABLED", false,
        "VISION_ENABLED", false,
        "ROLL_PITCH_SWAPPED", true
    );
    private static final Map<String, Object> OBJECT_CONSTANTS = Map.of(
        "NAVIGATION_CAMERA_TRANSFORM", new DynamicTransform3d(),
        "ARM_ORIGIN", null // TODO
    );
    public SquishyChassis() {}
    public Drivetrain getDrivetrain() {
        return new DrivetrainSquishy();
    }
    public double getDoubleConstant(String key) {
        return DOUBLE_CONSTANTS.get(key);
    }
    public String getStringConstant(String key)
    {
        return STRING_CONSTANTS.get(key);
    }
    public int getIntegerConstant(String key)
    {
        return INTEGER_CONSTANTS.get(key);
    }
    public boolean getBooleanConstant(String key)
    {
        return BOOLEAN_CONSTANTS.get(key);
    }
    public Object getObjectConstant(String key)
    {
        return OBJECT_CONSTANTS.get(key);
    }
    public String getChassisName() {
        return "Squishy";
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** 
 * Motor controller that can be constructed with followers.
 * Also interfaces with encoders.
 */
public interface MotorGroup extends MotorController {
    /**
     * Gives the rotations done by the encoder
     * @return double value for full rotations of encoder
     */
    public double getEncoderRotations();
    /**
     * Gets the rotational velocity of the encoder
     * @return double rotations per second
     */
    public double getEncoderRPS();
    /**
     * Sets a follower to have the opposite invert type as primary.
     * This must be called after followers are set.
     * @param index of the follower in parameter list.
     */
    public void setFollowerOppose(int i);
    /**
     * Resets the encoder rotations to (0,0)
     */
    public void resetEncoderRotations();
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.variants;

import java.util.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.subsystems.MotorGroup;

/** 
 * Group of Sparks to be used like MotorController Class
*/
public class MotorGroupSpark implements MotorGroup {
    private CANSparkMax primary;
    private List<CANSparkMax> followers = new ArrayList<CANSparkMax>();
    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     */
    public MotorGroupSpark(int primaryID, MotorType type) {
        this(primaryID, new int[]{}, type);
    }
    /**
     * Creates a new motor with followers controlled by talons.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     */
    public MotorGroupSpark(int primaryID, int[] followerIDs, MotorType type) {
        this.primary = new CANSparkMax(primaryID, type);
        for (int followerID: followerIDs) {
            this.followers.add(new CANSparkMax(followerID, type));
        }
        setFollowers();
        setInverted(false);
        configureAllControllers();
    }
    public void disable() {
        primary.disable();
    }
    public double get() {
        return primary.get();
    }
    public double getEncoderRotations() {
        return primary.getEncoder().getPosition();
    }
    public double getEncoderRPS() {
        // 1/60 represents the amount of 1 minute periods in a single second.
        primary.getEncoder().setVelocityConversionFactor(1/60);
        return primary.getEncoder().getVelocity();
    }
    public boolean getInverted() {
        return primary.getInverted();
    }
    public void set(double speed) {
        primary.set(speed);
    }
    public void setFollowerOppose(int i) {
        followers.get(i).follow(primary, true);
    }
    public void setInverted(boolean isInverted) {
        primary.setInverted(isInverted);
    }
    public void stopMotor() {
        primary.stopMotor();
    }
    public void resetEncoderRotations() {
        primary.getEncoder().setPosition(0);
    }
    private void configureAllControllers() {
        configureController(primary, false);
        for (CANSparkMax canSparkMax : followers) {
            configureController(canSparkMax, true);
        }
    }
    private void configureController(CANSparkMax controller, Boolean isFollower) {
        controller.setIdleMode(IdleMode.kBrake);
    }
    private void setFollowers() {
        for (CANSparkMax canSparkMax : followers) {
            canSparkMax.follow(primary);
        }
    }
}
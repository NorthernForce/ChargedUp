// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Motors;

import java.util.*;

import javax.naming.NameNotFoundException;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.jni.RevJNIWrapper;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Motors.MotorGroup;
/** 
 * Group of Sparks to be used like MotorController Class
*/
public class MotorGroupSpark extends CANSparkMax implements MotorGroup {
    private List<CANSparkMax> followers = new ArrayList<CANSparkMax>();
    private int invertCoefficient = 1;

    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     * @throws NameNotFoundException
     */
    public MotorGroupSpark(MotorType type, int primaryID) throws NameNotFoundException {
        this(type, primaryID, new int[]{});
    }

    /**
     * Creates a new motor with optional followers controlled by talons. 
     * Default inverted state is false.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     * @throws NameNotFoundException
     */
    public MotorGroupSpark(MotorType type, int primaryID, int... followerIDs) throws NameNotFoundException {
        super(primaryID, type);
        if (this.getFirmwareVersion() == 0) {
            throw new NameNotFoundException(String.format("The CAN ID %d is not found", primaryID));
        }

        for (int followerID: followerIDs) {
            this.followers.add(new CANSparkMax(followerID, type));
        }

        setFollowers();
        setInverted(false);
        configureAllControllers();
    }

    public double getEncoderRotations() {
        return invertCoefficient * this.getEncoder().getPosition();
    }

    public double getEncoderRPS() {
        // 1/60 represents the amount of 1 minute periods in a single second.
        this.getEncoder().setVelocityConversionFactor(1/60);
        return invertCoefficient * this.getEncoder().getVelocity();
    }

    public void setFollowerOppose(int i) {
        followers.get(i).follow(this, true);
    }
    
    public void setInverted(boolean isInverted) {
        super.setInverted(isInverted);
        invertCoefficient = (isInverted ? -1 : 1);
    }

    public void resetEncoderRotations() {
        this.getEncoder().setPosition(0);
    }

    private void configureAllControllers() {
        configureController(this, false);
        for (CANSparkMax canSparkMax : followers) {
            configureController(canSparkMax, true);
        }
    }

    private void configureController(CANSparkMax controller, Boolean isFollower) {
        controller.setIdleMode(IdleMode.kBrake);
    }

    private void setFollowers() {
        for (CANSparkMax canSparkMax : followers) {
            canSparkMax.follow(this);
        }
    }

    /**
     * Sets up soft limits for a spark max.
     * @param forward
     * @param backward
     */
    public void setLimits(Rotation2d forward, Rotation2d backward) {
        this.setSoftLimit(SoftLimitDirection.kForward, (float)backward.getDegrees());
        this.setSoftLimit(SoftLimitDirection.kReverse, (float)forward.getDegrees());
        this.enableSoftLimit(SoftLimitDirection.kForward, true);
        this.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void configurePID(int slot, double kP, double kI, double kD, double allowedCloseLoopErrors, double maxAccel, double maxVelocity, double minOutputVelocity){
        this.getPIDController().setP(kP, slot);
        this.getPIDController().setI(kI, slot);
        this.getPIDController().setD(kD, slot);
        this.getPIDController().setSmartMotionAllowedClosedLoopError(allowedCloseLoopErrors, slot);
        this.getPIDController().setSmartMotionMaxAccel(maxAccel, slot);
        this.getPIDController().setSmartMotionMaxVelocity(allowedCloseLoopErrors, slot);
        this.getPIDController().setSmartMotionMinOutputVelocity(minOutputVelocity, slot);
    }

    public void setUsingPID(double position, int slot){
        this.getPIDController().setReference(position, ControlType.kPosition, slot);
    }

    public void setUsingSmartMotion(double position, int slot){
        this.getPIDController().setReference(position, ControlType.kSmartMotion, slot);
    }
    
    public double getAbsolute(){
        return this.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
    
    public void resetAbsolute(double position){
        this.getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(position - getAbsolute());
    }

    public double getAbsoluteRPS() {
        return this.getAbsoluteEncoder(Type.kDutyCycle).getVelocity();
    }

    public void setFeedbackSensor(MotorFeedbackSensor sensor) {
        this.getPIDController().setFeedbackDevice(sensor);
    }

    public AbsoluteEncoder getAbsoluteEncoder() {
        return this.getAbsoluteEncoder(Type.kDutyCycle);
    }
}

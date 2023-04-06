// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Motors;

import java.util.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Motors.MotorGroup;
/** 
 * Group of Sparks to be used like MotorController Class
*/
public class MotorGroupSpark implements MotorGroup {
    private CANSparkMax primary;
    private List<CANSparkMax> followers = new ArrayList<CANSparkMax>();
    private int invertCoefficient = 1;
    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     */
    public MotorGroupSpark(MotorType type, int primaryID) {
        this(type, primaryID, new int[]{});
    }
    /**
     * Creates a new motor with optional followers controlled by talons. 
     * Default inverted state is false.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     */
    public MotorGroupSpark(MotorType type, int primaryID, int... followerIDs) {
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
        return invertCoefficient * primary.getEncoder().getPosition();
    }
    public double getEncoderRPS() {
        // 1/60 represents the amount of 1 minute periods in a single second.
        primary.getEncoder().setVelocityConversionFactor(1/60);
        return invertCoefficient * primary.getEncoder().getVelocity();
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
        invertCoefficient = (isInverted ? -1 : 1);
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
    /**
     * Sets up soft limits for a spark max.
     * @param forward
     * @param backward
     */
    public void setLimits(Rotation2d forward, Rotation2d backward) {
        primary.setSoftLimit(SoftLimitDirection.kForward, (float)backward.getDegrees());
        primary.setSoftLimit(SoftLimitDirection.kReverse, (float)forward.getDegrees());
        primary.enableSoftLimit(SoftLimitDirection.kForward, true);
        primary.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
    public void configurePID(int slot, double kP, double kI, double kD, double allowedCloseLoopErrors, double maxAccel, double maxVelocity, double minOutputVelocity){
        primary.getPIDController().setP(kP, slot);
        primary.getPIDController().setI(kI, slot);
        primary.getPIDController().setD(kD, slot);
        primary.getPIDController().setSmartMotionAllowedClosedLoopError(allowedCloseLoopErrors, slot);
        primary.getPIDController().setSmartMotionMaxAccel(maxAccel, slot);
        primary.getPIDController().setSmartMotionMaxVelocity(allowedCloseLoopErrors, slot);
        primary.getPIDController().setSmartMotionMinOutputVelocity(minOutputVelocity, slot);
    }
    public void setUsingPID(double position, int slot){
        primary.getPIDController().setReference(position, ControlType.kPosition, slot);
    }
    public double getAbsolute(){
        return primary.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
    public void resetAbsolute(double position){
            primary.getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(position - getAbsolute());
    }
    public double getAbsoluteRPS() {
        return primary.getAbsoluteEncoder(Type.kDutyCycle).getVelocity();
    }
    public void setFeedbackSensor(MotorFeedbackSensor sensor)
    {
        primary.getPIDController().setFeedbackDevice(sensor);
    }
    public AbsoluteEncoder getAbsoluteEncoder()
    {
        return primary.getAbsoluteEncoder(Type.kDutyCycle);
    }
    @Override
    public void setSpeed(double speed)
    {
        primary.getPIDController().setReference(speed, ControlType.kVelocity);
    }
}

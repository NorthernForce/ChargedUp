// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.variants;

import java.util.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.*;
import frc.robot.subsystems.MotorGroup;


/** 
 * Group of Talons to be used like MotorController Class
*/
public class MotorGroupTalonSRX implements MotorGroup {
    private WPI_TalonSRX primary;
    private List<WPI_TalonSRX> followers = new ArrayList<WPI_TalonSRX>();
    private int COUNTS_PER_REVOLUTION = 2048;
    private int invertCoefficient = 1;
    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     */
    public MotorGroupTalonSRX(int primaryID) {
        this(primaryID, new int[]{});
    }
    /**
     * Creates a new motor with optional followers controlled by talons.
     * Default inverted state is false.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     */
    public MotorGroupTalonSRX(int primaryID, int[] followerIDs) {
        this.primary = new WPI_TalonSRX(primaryID);
        for (int followerID: followerIDs) {
            this.followers.add(new WPI_TalonSRX(followerID));
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
        return primary.getSelectedSensorPosition() / COUNTS_PER_REVOLUTION;
    }
    public double getEncoderRPS() {
        //10 represents the amount of 100ms periods in a single second.
        return primary.getSelectedSensorVelocity() / COUNTS_PER_REVOLUTION * 10;
    }
    public boolean getInverted() {
        return primary.getInverted();
    }
    public void set(double speed) {
        primary.set(speed);
    }
    public void setFollowerOppose(int i) {
        followers.get(i).setInverted(InvertType.OpposeMaster);
    }
    public void setInverted(boolean isInverted) {
        primary.setInverted(isInverted);
        invertCoefficient = (isInverted ? -1 : 1);
    }
    public void stopMotor() {
        primary.stopMotor();
    }
    public void resetEncoderRotations() {
        primary.setSelectedSensorPosition(0);
    }
    private void configureAllControllers() {
        configureController(primary, false);
        for (WPI_TalonSRX wpi_TalonFX : followers) {
            configureController(wpi_TalonFX, true);
        }
    }
    /**
     * Sets the velocity of the motor
     * Uses integrated PID.
     * @param velocity does not factor in gear ratio
     * @param feedforward value of feedforward
     */
    public void setVelocity(double velocity, double feedforward)
    {
        primary.set(ControlMode.Velocity, velocity * COUNTS_PER_REVOLUTION / 10, DemandType.ArbitraryFeedForward, feedforward);
    }
    /**
     * Sets the position using motion magic
     * @param position position in rotations... does not factor in gear ratio
     * @param feedforward the feedforward value to be added
     */
    public void setMotionMagic(double position, double feedforward)
    {
        primary.set(ControlMode.MotionMagic, position * COUNTS_PER_REVOLUTION, DemandType.ArbitraryFeedForward, feedforward);
    }
    public void setCountsPerRevolution(int countsPerRevolution)
    {
        COUNTS_PER_REVOLUTION = countsPerRevolution;
    }
    /**
     * Sets the position of Falcon motor using integrated PIDControl
     * @param rotations Number of rotations. Does not factor in gear ratio.
     */
    public void setPosition(double rotations)
    {
        primary.set(ControlMode.Position, rotations * COUNTS_PER_REVOLUTION);
    }
    /**
     * Configures a closed loop
     * @param slotIdx the index of the closed loop to configure. Thus you can have multiple
     * @param allowableError allowableError in sensor units per 100ms.
     * @param kF velocity feedforward gain
     * @param kP proportion
     * @param kI integral
     * @param kD derivative
     */
    public void configClosedLoop(int slotIdx, double allowableError, double kF, double kP, double kI, double kD)
    {
        primary.configAllowableClosedloopError(slotIdx, allowableError, 0);
        primary.config_kF(slotIdx, kF, 0);
		primary.config_kP(slotIdx, kP, 0);
		primary.config_kI(slotIdx, kI, 0);
		primary.config_kD(slotIdx, kD, 0);
    }
    /**
     * Sets the current encoder rotations
     * @param rotations in rotations... does not factor in gear ratio
     */
    public void setEncoderRotations(double rotations)
    {
        primary.setSelectedSensorPosition(COUNTS_PER_REVOLUTION * rotations);
    }
    private void configureController(WPI_TalonSRX controller, Boolean isFollower) {
        /** These 3 values are used to prevent breakers from tripping*/
        final double currentLimit = 40; //Holding current in amps to limit when feature is activated
        final double limitThreshold = 90; //Current must excede this threshold (amps) before limiting occurs
        final double triggerThreshTimeInSec = 1; //How long the current must excede thrreshold (seconds) before limiting occurs
        controller.configFactoryDefault();
        controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, limitThreshold, triggerThreshTimeInSec));
        if (!isFollower) {
          controller.configClosedloopRamp(DRIVE_RAMP_RATE);
          controller.configOpenloopRamp(DRIVE_RAMP_RATE);
        }
        controller.setNeutralMode(NeutralMode.Brake);
        TalonSRXConfiguration configs = new TalonSRXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        controller.configAllSettings(configs);
    }
    private void setFollowers() {
        for (WPI_TalonSRX wpi_TalonFX : followers) {
            wpi_TalonFX.follow(primary);
            wpi_TalonFX.setInverted(InvertType.FollowMaster);
        }
    }
}

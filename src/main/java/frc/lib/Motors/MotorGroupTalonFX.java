// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Motors;

import java.util.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.platform.DeviceType;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;


/** 
 * Group of Talons to be used like MotorController Class
*/
public class MotorGroupTalonFX implements MotorGroup {
    private WPI_TalonFX primary;
    private List<WPI_TalonFX> followers = new ArrayList<WPI_TalonFX>();
    private int COUNTS_PER_REVOLUTION = 2048;
    private int invertCoefficient = 1;
    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     */
    public MotorGroupTalonFX(int primaryID) {
        this(primaryID, new int[]{});
    }
    /**
     * Creates a new motor with optional followers controlled by talons.
     * Default inverted state is false.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     */
    public MotorGroupTalonFX(int primaryID, int... followerIDs) {
        this.primary = new WPI_TalonFX(primaryID);
        for (int followerID: followerIDs) {
            this.followers.add(new WPI_TalonFX(followerID));
        }
        setFollowers();
        setInverted(false);
        configureAllControllers();
    }
    public void setCountsPerRevolution(int countsPerRevolution)
    {
        COUNTS_PER_REVOLUTION = countsPerRevolution;
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
        return invertCoefficient * primary.getSelectedSensorVelocity() / COUNTS_PER_REVOLUTION * 10;
    }
    public boolean getInverted() {
        return primary.getInverted();
    }
    public void set(double speed) {
        primary.set(speed);
    }
    /**
     * Sets the position of Falcon motor using integrated PIDControl
     * @param rotations Number of rotations. Does not factor in gear ratio.
     */
    public void setPosition(double rotations, double feedforward)
    {
        primary.set(ControlMode.Position, rotations * COUNTS_PER_REVOLUTION, DemandType.ArbitraryFeedForward, feedforward);
    }
    public void setPercent(double percent, double feedforward)
    {
        primary.set(ControlMode.PercentOutput, percent, DemandType.ArbitraryFeedForward, feedforward);
    }
    /*
     * Sets the position using motion magic
     * @param position position in rotations... does not factor in gear ratio
     * @param feedforward the feedforward value to be added
     */
    public void setMotionMagic(double position, double feedforward)
    { 
        primary.set(ControlMode.MotionMagic, position * COUNTS_PER_REVOLUTION, DemandType.ArbitraryFeedForward, feedforward);
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
    /**
     * Links and selects the cancoder
     * @param coder the CANCoder reference
     */
    public void linkAndUseCANCoder(CANCoder coder)
    {
        primary.configRemoteFeedbackFilter(coder, 0);
        primary.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        COUNTS_PER_REVOLUTION = 4096;
    }
    /*
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
    public void configSelectedProfile(int slotIdx, int pidIdx)
    {
        primary.selectProfileSlot(slotIdx, pidIdx);
    }
    private void configureAllControllers() {
        configureController(primary, false);
        for (WPI_TalonFX wpi_TalonFX : followers) {
            configureController(wpi_TalonFX, true);
        }
    }
    private void configureController(WPI_TalonFX controller, Boolean isFollower) {
        /** These 3 values are used to prevent breakers from tripping*/
        final double currentLimit = 40; //Holding current in amps to limit when feature is activated
        final double limitThreshold = 90; //Current must excede this threshold (amps) before limiting occurs
        final double triggerThreshTimeInSec = 1; //How long the current must excede thrreshold (seconds) before limiting occurs
        controller.configFactoryDefault();
        controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, limitThreshold, triggerThreshTimeInSec));
        if (!isFollower) {
          controller.configClosedloopRamp(Constants.DrivetrainConstants.RAMP_RATE);
          controller.configOpenloopRamp(Constants.DrivetrainConstants.RAMP_RATE);
        }
        controller.setNeutralMode(NeutralMode.Brake);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        controller.configAllSettings(configs);
    }
    private void setFollowers() {
        for (WPI_TalonFX wpi_TalonFX : followers) {
            wpi_TalonFX.follow(primary);
            wpi_TalonFX.setInverted(InvertType.FollowMaster);
        }
    }
    /**
     * Gets iteratable of all of the motors, including the primary.
     */
    public List<WPI_TalonFX> getAllMotors()
    {
        ArrayList<WPI_TalonFX> list = new ArrayList<WPI_TalonFX>();
        list.add(primary);
        for (var follower : followers)
        {
            list.add(follower);
        }
        return list;
    }
    public void setLimits(Rotation2d forward, Rotation2d reverse) {
        primary.configForwardSoftLimitThreshold(forward.getRotations() * COUNTS_PER_REVOLUTION, 0);
        primary.configReverseSoftLimitThreshold(reverse.getRotations() * COUNTS_PER_REVOLUTION, 0);
        primary.configForwardSoftLimitEnable(true, 0);
        primary.configReverseSoftLimitEnable(true, 0);
    }
}

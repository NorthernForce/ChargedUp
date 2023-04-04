// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.Intake;
import frc.robot.commands.LEDAdjustment;
import frc.robot.commands.ManipulateCone;
import frc.robot.commands.ManipulateCube;
import frc.robot.commands.Outtake;
import frc.robot.commands.RetractArm;
import frc.robot.commands.ToggleLED;
import frc.robot.commands.autoComponents.TurnToTarget;


/** Add your docs here. */
public class OI {
    public static final XboxController driverController = new XboxController(0);
    public static final XboxController manipulatorController = new XboxController(1);
    public OI() {}
    /**
     * Gets the joystick controls from the drivercontroller we use to control the robots drivetrain
     * @return Index[0]: Joystick forward(positve)/back(negative).
     * <li> Index[1]: Joystick left(positive)/right(negative). </li>
     */
    public static DoubleSupplier[] getDriveSuppliers() {
            return new DoubleSupplier[] {
            //Joystick defaults to forward being negative so we negate it.
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()
        };
    }
    /**
     * Gets two double suppliers representing the two major axis of control on manipulator controller.
     * @return two doubles that will be within [-1.0, 1.0]
     */
    public static DoubleSupplier[] getManipulatorSuppliers() {
            return new DoubleSupplier[] {
            () -> -manipulatorController.getLeftY(),
            () -> -manipulatorController.getRightX(),
            () -> Math.abs(manipulatorController.getRightY()) > 0.15 ? -manipulatorController.getRightY() : 0
        };
    }
    /** Binds the buttons of the OI */
    public void bindButtons() {
        new JoystickButton(manipulatorController, XboxController.Button.kRightBumper.value)
            .onTrue(new ExtendArm());
        new JoystickButton(manipulatorController, XboxController.Button.kLeftBumper.value)
            .onTrue(new RetractArm());
        new Trigger(() -> manipulatorController.getRightTriggerAxis() > 0.2) // gripper trigger sensitivity
            .whileTrue(new Outtake());
        new Trigger(() -> manipulatorController.getLeftTriggerAxis() > 0.2) // gripper trigger sensitivity
            .whileTrue(new Intake());
        new JoystickButton(manipulatorController, XboxController.Button.kY.value)
            .whileTrue(new ManipulateCone());
        new JoystickButton(manipulatorController, XboxController.Button.kX.value)
            .whileTrue(new ManipulateCube());
        new JoystickButton(manipulatorController, XboxController.Button.kA.value)
            .toggleOnTrue(new ToggleLED());
        new Trigger(() -> Math.abs(RobotContainer.armRotate.getAngle().getDegrees() - 90) < 5)
            .onTrue(new RumbleManipulator());                                                               
        new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
            .onTrue(new LEDAdjustment(10));            
        // As per their request, the micro adjustment is on both controllers
        new JoystickButton(driverController, XboxController.Button.kBack.value)
            .whileTrue(new MicroAdjust(Constants.DrivetrainConstants.LEFT_MIRCO_ADJUST));
        new JoystickButton(driverController, XboxController.Button.kStart.value)
            .whileTrue(new MicroAdjust(Constants.DrivetrainConstants.RIGHT_MIRCO_ADJUST));
        new JoystickButton(manipulatorController, XboxController.Button.kBack.value)
            .whileTrue(new MicroAdjust(Constants.DrivetrainConstants.LEFT_MIRCO_ADJUST));
        new JoystickButton(manipulatorController, XboxController.Button.kStart.value)
            .whileTrue(new MicroAdjust(Constants.DrivetrainConstants.RIGHT_MIRCO_ADJUST));    
        new Trigger(() -> manipulatorController.getPOV() == 0)
            .whileTrue(new SetArmAngle(Constants.ArmConstants.NORTH_ANGLE).alongWith(new SetWristAngle(Constants.WristConstants.NORTH_ANGLE)));
        new JoystickButton(manipulatorController, XboxController.Button.kB.value)
        .whileTrue(new SetArmAngle(Constants.ArmConstants.EAST_ANGLE).alongWith(new SetWristAngle(Constants.WristConstants.EAST_ANGLE)));
        new Trigger(() -> manipulatorController.getPOV() == 180)
        .whileTrue(new SetArmAngle(Constants.ArmConstants.SOUTH_ANGLE).alongWith(new SetWristAngle(Constants.WristConstants.SOUTH_ANGLE)));
        new Trigger(() -> manipulatorController.getPOV() == 270)
        .whileTrue(new SetArmAngle(Constants.ArmConstants.WEST_ANGLE).alongWith(new SetWristAngle(Constants.WristConstants.WEST_ANGLE)));
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
            .whileTrue(Commands.runOnce(() -> RobotContainer.vision.setPipeline(0, 2)).andThen(new TurnToTarget(0)))
            .onFalse(Commands.runOnce(() -> RobotContainer.vision.setPipeline(0, 1)));
    }
}

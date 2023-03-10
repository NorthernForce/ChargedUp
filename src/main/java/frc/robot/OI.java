// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.RunConeIntake;
import frc.robot.commands.RunConeOuttake;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.autoComponents.TurnToCone;
import frc.robot.commands.autoComponents.TurnToCube;
import frc.robot.commands.autoComponents.TurnToReflectiveTape;

import frc.robot.commands.LEDPurple;
import frc.robot.commands.LEDYellow;
import frc.robot.commands.RetractArm;

/** Add your docs here. */
public class OI {
    private static final XboxController driverController = new XboxController(0);
    private static final XboxController manipulatorController = new XboxController(1);
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
            () -> manipulatorController.getRightX()
        };
    }
    /** Binds the buttons of the OI */
    public void bindButtons() {
        if (Constants.COMPRESSOR_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Axis.kRightTrigger.value)
                .onTrue(new ExtendArm());
            new JoystickButton(manipulatorController, XboxController.Axis.kRightTrigger.value)
                .onTrue(new RetractArm());
        }
        if (Constants.GRIPPER_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Button.kA.value)
                .whileTrue(new RunConeIntake());
            new JoystickButton(manipulatorController, XboxController.Button.kB.value)
                .whileTrue(new RunConeOuttake());
        }
        if (Constants.LED_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Button.kY.value)
                .whileTrue(new LEDPurple());
        }
        if (Constants.LED_ENABLED)
        {
            new JoystickButton(manipulatorController, XboxController.Button.kX.value)
                .whileTrue(new LEDYellow());
        }
    }
}

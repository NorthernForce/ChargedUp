// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.Intake;
import frc.robot.commands.ManipulateCone;
import frc.robot.commands.ManipulateCube;
import frc.robot.commands.Outtake;
import frc.robot.commands.RetractArm;
import frc.robot.commands.ToggleLED;
import frc.robot.commands.RumbleManipulator;


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
            () -> -manipulatorController.getRightY()
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
    }
}

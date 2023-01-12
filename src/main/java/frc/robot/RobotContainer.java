// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.PositioningSubsystem;
import frc.robot.commands.CalibrateIMU;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ArrayList<Pair<PhotonCamera, Transform3d>> cameras = new ArrayList<>();
  {
    // TODO: add cameras
  }
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final IMU imu = new IMU();
  public static final PositioningSubsystem positioningSubsystem = new PositioningSubsystem(null);
  private final SendableChooser<Command> autonomousChooser;

  private final OI oi = new OI();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    initDefaultCommands();
    oi.bindButtons();
    autonomousChooser = new SendableChooser<>();
    autonomousChooser.addOption("Instant Command(Do nothing)", new InstantCommand());
    SmartDashboard.putData("Autonomous Routine Chooser", autonomousChooser);
    SmartDashboard.putData("Calibrate IMU", new CalibrateIMU());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomousChooser.getSelected();
  }

  private void initDefaultCommands() {
    drivetrain.setDefaultCommand(new DriveWithJoystick());
  }
  public void periodic()
  {
    SmartDashboard.putNumber("Forward Speed Proportion", drivetrain.getSpeedProportion());
    SmartDashboard.putNumber("Rotation Speed Proportion", drivetrain.getRotationSpeedProportion());
  }
}

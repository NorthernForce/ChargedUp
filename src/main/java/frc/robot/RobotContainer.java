package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.LEDInit;
import frc.robot.commands.ManipulateArmWithJoystick;
import frc.robot.commands.autoComponents.*;
import frc.robot.commands.autoPaths.*;
import frc.robot.commands.CalibrateIMU;
import frc.robot.commands.DefaultWrist;
import frc.robot.util.RobotChooser;
import frc.robot.chassis.ChassisBase;
import frc.robot.subsystems.*;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

  public static final ChassisBase activeChassis = new RobotChooser().GetChassis();
  public static final ArmRotate armRotate = Constants.ARM_ENABLED ? new ArmRotate() : null;
  public static final ArmTelescope armTelescope = Constants.ARM_ENABLED ? new ArmTelescope() : null;
  public static final PCM pcm = Constants.COMPRESSOR_ENABLED ? new PCM() : null;
  public static final Drivetrain drivetrain = Constants.DRIVETRAIN_ENABLED ? activeChassis.getDrivetrain() : null;
  public static final Gripper gripper = Constants.GRIPPER_ENABLED ? new Gripper() : null;
  public static final IMU imu = Constants.IMU_ENABLED ? new IMU() : null;
  public static final LED led = Constants.LED_ENABLED ? new LED() : null;
  public static final Navigation navigation = Constants.NAVIGATION_ENABLED ? new Navigation() : null;
  public static final Vision vision = Constants.VISION_ENABLED ? new Vision() : null;
  public static final Wrist wrist = Constants.WRIST_ENABLED ? new Wrist() : null;
  private final SendableChooser<Command> auto1, auto2, auto3;
  private final OI oi = new OI();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    PhotonCamera.setVersionCheckEnabled(false);
    initDefaultCommands();
    oi.bindButtons();

    auto1 = new SendableChooser<>();
    auto1.addOption("Do nothing", new InstantCommand());
    auto1.addOption("Roll Cube off", new PushOffPiece());
    auto1.addOption("Place Upper left", new InstantCommand());
    auto1.addOption("Place Upper Center", new InstantCommand());
    auto1.addOption("Place Upper Right", new InstantCommand());
    auto1.addOption("Place Middle left", new InstantCommand());
    auto1.addOption("Place Middle Center", new InstantCommand());
    auto1.addOption("Place Middle Right", new InstantCommand());
    auto1.addOption("Place Lower left", new InstantCommand());
    auto1.addOption("Place Upper Center", new InstantCommand());
    auto1.addOption("Place Upper Right", new InstantCommand());

    auto2 = new SendableChooser<>();
    auto2.addOption("Do Nothing", new InstantCommand());
    auto2.addOption("Mobility", new ExitCommunity());
    auto2.addOption("Engage", new InstantCommand());
    auto2.addOption("Pick up P1", new InstantCommand());
    auto2.addOption("Pick up P2", new InstantCommand());
    auto2.addOption("Pick up P3", new InstantCommand());
    auto2.addOption("Pick up P4", new InstantCommand());

    auto3 = new SendableChooser<>();
    auto3.addOption("Do Nothing", new InstantCommand());
    auto3.addOption("Engage", new InstantCommand());
    auto3.addOption("Place Upper left", new InstantCommand());
    auto3.addOption("Place Upper Center", new InstantCommand());
    auto3.addOption("Place Upper Right", new InstantCommand());
    auto3.addOption("Place Middle left", new InstantCommand());
    auto3.addOption("Place Middle Center", new InstantCommand());
    auto3.addOption("Place Middle Right", new InstantCommand());
    auto3.addOption("Place Lower left", new InstantCommand());
    auto3.addOption("Place Upper Center", new InstantCommand());
    auto3.addOption("Place Upper Right", new InstantCommand());

    SmartDashboard.putData("Autonomous 1", auto1);
    SmartDashboard.putData("Autonomous 2", auto2);
    SmartDashboard.putData("Autonomous 3", auto3);
    SmartDashboard.putData("Calibrate IMU", new CalibrateIMU());
    SmartDashboard.putData("Stop", new Stop(0.1));
    SmartDashboard.putData("PID Balance", new PIDBalance());
    SmartDashboard.putString("Robot Name: ", activeChassis.getChassisName());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(auto1.getSelected(), auto2.getSelected(), auto3.getSelected(), new Stop(0.1));

  }
  /** Initializes the default commands for each subsystem */
  private void initDefaultCommands() {
    if (Constants.DRIVETRAIN_ENABLED) drivetrain.setDefaultCommand(new DriveWithJoystick());
    if (Constants.ARM_ENABLED) armRotate.setDefaultCommand(new ManipulateArmWithJoystick());
    if (Constants.WRIST_ENABLED) wrist.setDefaultCommand(new DefaultWrist());
    if (Constants.LED_ENABLED) led.setDefaultCommand(new LEDInit());
  }
  public void periodic() {}
}

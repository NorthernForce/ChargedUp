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

import java.io.IOException;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
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
  private final SendableChooser<Command> autonomousChooser;
  private final SendableChooser<Pose2d> startingLocationChooser;
  private final OI oi = new OI();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    PhotonCamera.setVersionCheckEnabled(false);
    initDefaultCommands();
    oi.bindButtons();
    autonomousChooser = new SendableChooser<>();
    autonomousChooser.addOption("Instant Command(Do nothing)", new InstantCommand());
    autonomousChooser.addOption("Human Grid. Mobility", new HG_Mob());
    autonomousChooser.setDefaultOption("Outer Grid. 1 piece mobility", new OG_1PieMob());
    autonomousChooser.addOption("Center. Mob. Balance", new CG_Mob_E());
    try
    {
      autonomousChooser.addOption("Red1 to Piece1", new DriveAlongPath("Red1ToPiece1")
        .andThen(new Stop(0.1))
        .andThen(new DriveAlongPath("Piece1ToRed1"))
        .andThen(new Stop(0.1)));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    startingLocationChooser = new SendableChooser<>();
    startingLocationChooser.setDefaultOption("Red Left", Constants.RED_POSES[0]);
    startingLocationChooser.addOption("Red Center", Constants.RED_POSES[1]);
    startingLocationChooser.addOption("Red Right", Constants.RED_POSES[2]);
    startingLocationChooser.addOption("Blue Left", Constants.BLUE_POSES[0]);
    startingLocationChooser.addOption("Blue Center", Constants.BLUE_POSES[1]);
    startingLocationChooser.addOption("Blue Right", Constants.BLUE_POSES[2]);
    SmartDashboard.putData("Autonomous Routine Chooser", autonomousChooser);
    SmartDashboard.putData("Starting Location Chooser", startingLocationChooser);
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
  public Command getAutonomousCommand() {
    if (Constants.NAVIGATION_ENABLED) navigation.setRobotPose(startingLocationChooser.getSelected());
    // An ExampleCommand will run in autonomous

    return autonomousChooser.getSelected();
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

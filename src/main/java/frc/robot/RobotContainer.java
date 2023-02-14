package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ManipulateArmWithJoystick;
import frc.robot.commands.autoComponents.*;
import frc.robot.commands.CalibrateIMU;

import frc.robot.util.RobotChooser;
import frc.robot.chassis.ChassisBase;
import frc.robot.subsystems.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final Arm arm = Constants.ARM_ENABLED ? new Arm() : null;
  public static final PCM pcm = Constants.COMPRESSOR_ENABLED ? new PCM() : null;
  public static final Drivetrain drivetrain = Constants.DRIVETRAIN_ENABLED ? activeChassis.getDrivetrain() : null;
  public static final Gripper gripper = Constants.GRIPPER_ENABLED ? new Gripper() : null;
  public static final IMU imu = Constants.IMU_ENABLED ? new IMU() : null;
  public static final LED led = Constants.LED_ENABLED ? new LED() : null;
  public static final Navigation navigation = Constants.NAVIGATION_ENABLED ? new Navigation() : null;
  public static final Vision vision = Constants.VISION_ENABLED ? new Vision() : null;
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
    autonomousChooser.addOption("Drive to Point", new DriveToLocation(
      new Pose2d(
        13.07,
        1.09,
        new Rotation2d(Math.toRadians(180))
    )));
    autonomousChooser.addOption("Blue #1", new SequentialCommandGroup(
      new DriveToLocation(null)
    ));
    SmartDashboard.putData("Autonomous Routine Chooser", autonomousChooser);
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
    // An ExampleCommand will run in autonomous
    return autonomousChooser.getSelected();
  }
  /** Initializes the default commands for each subsystem */
  private void initDefaultCommands() {
    if (Constants.DRIVETRAIN_ENABLED) drivetrain.setDefaultCommand(new DriveWithJoystick());
    if (Constants.ARM_ENABLED) arm.setDefaultCommand(new ManipulateArmWithJoystick());
  }
  public void periodic()
  {
  }
}
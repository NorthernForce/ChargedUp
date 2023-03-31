package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ManipulateArmWithJoystick;
import frc.robot.commands.autoComponents.*;
import frc.robot.commands.autoPaths.*;
import frc.robot.states.manipulatingstate.CubeManipulatingState;
import frc.robot.states.manipulatingstate.EmptyManipulatingState;
import frc.robot.states.manipulatingstate.ManipulatingState;
import frc.robot.states.manipulatingstate.ManipulatingStateContainer;
import frc.robot.commands.CalibrateArm;
import frc.robot.commands.CalibrateIMU;
import frc.robot.commands.CalibrateWrist;
import frc.robot.commands.DefaultWrist;
import frc.robot.util.RobotChooser;
import frc.robot.chassis.ChassisBase;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  /** State objects */
  public static final ChassisBase activeChassis = RobotChooser.getChassis();
  public static final ManipulatingStateContainer manipulatingState = new ManipulatingStateContainer();

  /** Subsystem Objects */
  public static final ArmRotate armRotate =  new ArmRotate();
  public static final ArmTelescope armTelescope = new ArmTelescope();
  public static final PCM pcm = new PCM();
  public static final Drivetrain drivetrain = activeChassis.getDrivetrain();
  public static final Gripper gripper = new Gripper();
  public static final IMU imu = new IMU();
  public static final LED led = new LED();
  public static final Navigation navigation = new Navigation();
  public static final Vision vision = new Vision();
  public static final Wrist wrist = new Wrist();

  /** Private Objects */
  private final SendableChooser<Command> autonomousChooser;
  private final SendableChooser<Pose2d> startingLocationChooser;
  private final OI oi = new OI();
  private PowerDistribution pdh = new PowerDistribution(21, ModuleType.kRev);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    initDefaultCommands();
    oi.bindButtons();
    autonomousChooser = new SendableChooser<>();
    autonomousChooser.addOption("Instant Command(Do nothing)", new InstantCommand());
    autonomousChooser.addOption("Human Grid. Mobility", new HG_Mob());
    autonomousChooser.setDefaultOption("Outer Grid. 1 piece mobility", new OG_1PieMob());
    autonomousChooser.addOption("Center. Mob. Balance", new CG_Mob_E());
    autonomousChooser.addOption("Blue Center 1 Piece Balance", new BlueCenter());
    autonomousChooser.addOption("Red Center 1 Piece Balance", new RedCenter());
    autonomousChooser.addOption("Red Center Balance Only", new RedOnlyCenter());
    autonomousChooser.addOption("Blue Center Balance Only", new BlueOnlyCenter());
    //autonomousChooser.addOption("Temporary Test", new FoldWristBack());
    /**
     * An IOException occurs when you access files that cause errors of a sort.
     * Each path is loaded from a file, therefore there is a risk of an IOException.
     * Instead of breaking the robot code when the robot runs, the IOException is caught and the stack trace is printed.
     */
    try
    {
      autonomousChooser.addOption("Red Left 1 Piece", new RedLeft(1));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Red Right 1 Piece", new RedRight(1));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Blue Left 1 Piece", new BlueLeft(1));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Blue Right 1 Piece", new BlueRight(1));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Red Left 2 Piece", new RedLeft(2));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Red Right 2 Piece", new RedRight(2));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Blue Left 2 Piece", new BlueLeft(2));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Blue Right 2 Piece", new BlueRight(2));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Red Left 3 Piece", new RedLeft(3));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Red Right 3 Piece", new RedRight(3));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Blue Left 3 Piece", new BlueLeft(3));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    try
    {
      autonomousChooser.addOption("Blue Right 3 Piece", new BlueRight(3));
    }
    catch (IOException exception)
    {
      exception.printStackTrace();
    }
    startingLocationChooser = new SendableChooser<>();
    startingLocationChooser.setDefaultOption("Red Left", FieldConstants.RED_POSES[0]);
    startingLocationChooser.addOption("Red Center", FieldConstants.RED_POSES[1]);
    startingLocationChooser.addOption("Red Right", FieldConstants.RED_POSES[2]);
    startingLocationChooser.addOption("Blue Left", FieldConstants.BLUE_POSES[0]);
    startingLocationChooser.addOption("Blue Center", FieldConstants.BLUE_POSES[1]);
    startingLocationChooser.addOption("Blue Right", FieldConstants.BLUE_POSES[2]);
    Shuffleboard.getTab("Autonomous").add("Autonomous Routine Chooser", autonomousChooser).withSize(2, 1).withPosition(2, 2);
    Shuffleboard.getTab("Autonomous").add("Starting Location Chooser", startingLocationChooser).withSize(2, 1).withPosition(0, 2);
    Shuffleboard.getTab("Utility").add("Calibrate IMU", new CalibrateIMU()).withPosition(0, 0);
    Shuffleboard.getTab("Utility").add("Stop", new Stop(0.1)).withPosition(1, 0);
    Shuffleboard.getTab("Utility").add("Balance", new Balance()).withPosition(2, 0);
    Shuffleboard.getTab("Utility").add("Robot Name: ", activeChassis.getChassisName()).withPosition(0, 1);
    Shuffleboard.getTab("Utility").addNumber("Current Draw (Amp)", () -> pdh.getTotalCurrent());
    Shuffleboard.getTab("Utility").add("Calibrate Wrist", new CalibrateWrist());
    Shuffleboard.getTab("Arm").add("Calibrate Arm", new CalibrateArm());
    Shuffleboard.getTab("Drivers").addBoolean("Manipulating State", () -> manipulatingState.getCurrentState().getClass().equals(CubeManipulatingState.class))
      .withPosition(5, 1)
      .withProperties(Map.of("colorWhenTrue", "purple", "colorWhenFalse", "yellow"));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    navigation.setRobotPose(startingLocationChooser.getSelected());
    // An ExampleCommand will run in autonomous

    return autonomousChooser.getSelected();
  }
  /** Initializes the default commands for each subsystem */
  private void initDefaultCommands() {
    drivetrain.setDefaultCommand(new DriveWithJoystick());
    armRotate.setDefaultCommand(new ManipulateArmWithJoystick());
    wrist.setDefaultCommand(new DefaultWrist());
  }
  public void periodic() {
  }
}

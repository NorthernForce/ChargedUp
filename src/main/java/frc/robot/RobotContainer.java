package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.LEDInit;
import frc.robot.commands.ManipulateArmWithJoystick;
import frc.robot.commands.ManipulateCube;
import frc.robot.commands.Outtake;
import frc.robot.commands.RunConeOuttake;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.autoComponents.*;
import frc.robot.commands.autoPaths.*;
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
import static frc.robot.Constants.WristConstants.CUBE_OVERSHOOT;
import static frc.robot.Constants.WristConstants.FLOOR_CUBE_PICKUP_ANGLE;
import static frc.robot.Constants.WristConstants.FLOOR_CUBE_PLACEMENT_ANGLE;
import static frc.robot.Constants.WristConstants.HIGH_CUBE_PLACEMENT_ANGLE;
import static frc.robot.Constants.WristConstants.MID_CUBE_PLACEMENT_ANGLE;
import static frc.robot.FieldConstants.RED_CUBE_PLACEMENT_LOCATIONS;
import static frc.robot.FieldConstants.RED_FLOOR_PLACEMENT_LOCATIONS;
import static frc.robot.FieldConstants.*;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.AnglesAndDistances;
import static frc.robot.FieldConstants.*;
import frc.robot.Constants;
import static frc.robot.Constants.ArmConstants;
import static frc.robot.Constants.GripperConstants;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.PiceConstants;
import static frc.robot.Constants.AnglesAndDistances;
import static frc.robot.FieldConstants.*;
import frc.robot.Constants.AnglesAndDistances;
import frc.robot.chassis.ChassisBase;
import frc.robot.commands.CalibrateArm;
import frc.robot.commands.CalibrateIMU;
import frc.robot.commands.CalibrateWrist;
import frc.robot.commands.DefaultWrist;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.LEDInit;
import frc.robot.commands.ManipulateArmWithJoystick;
import frc.robot.commands.autoComponents.AutoIntake;
import frc.robot.commands.autoComponents.AutoOuttake;
import frc.robot.commands.autoComponents.Balance;
import frc.robot.commands.autoComponents.DriveAlongPath;
import frc.robot.commands.autoComponents.PositionWithTarget;
import frc.robot.commands.autoComponents.Stop;
import frc.robot.commands.autoPaths.CG_Mob_E;
import frc.robot.commands.autoPaths.HG_Mob;
import frc.robot.commands.autoPaths.OG_1PieMob;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.ArmTelescope;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.PCM;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.util.RobotChooser;
import frc.robot.Constants;
import static frc.robot.Constants.ArmConstants;
import static frc.robot.Constants.GripperConstants;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.PiceConstants;
import static frc.robot.Constants.AnglesAndDistances;
import static frc.robot.FieldConstants.*;

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
    /**
     * An IOException occurs when you access files that cause errors of a sort.
     * Each path is loaded from a file, therefore there is a risk of an IOException.
     * Instead of breaking the robot code when the robot runs, the IOException is caught and the stack trace is printed.
     */
    try
    {
      autonomousChooser.addOption("Red Right 2 Piece", 
        new PositionWithTarget(RED_CUBE_PLACEMENT_LOCATIONS[6].toTranslation2d(), AnglesAndDistances.HIGH_CUBE.getFirst(), AnglesAndDistances.HIGH_CUBE.getSecond().plus(CUBE_OVERSHOOT), HIGH_CUBE_PLACEMENT_ANGLE, true)
        .andThen(new AutoOuttake())
        .andThen(new DriveAlongPath("Red1ToPiece1"))
        .andThen(new Stop(0.1))
        .andThen(new PositionWithTarget(RED_GAME_PIECE_AUTO_LOCATIONS[3].toTranslation2d(), AnglesAndDistances.FLOOR_CUBE.getFirst(), AnglesAndDistances.FLOOR_CUBE.getSecond().plus(CUBE_OVERSHOOT), FLOOR_CUBE_PICKUP_ANGLE, false))
        .andThen(new AutoIntake())
        .andThen(new DriveAlongPath("Piece1ToRed1"))
        .andThen(new Stop(0.1))
        .andThen(new PositionWithTarget(RED_CUBE_PLACEMENT_LOCATIONS[5].toTranslation2d(), AnglesAndDistances.MEDIUM_CUBE.getFirst(), AnglesAndDistances.MEDIUM_CUBE.getSecond(), MID_CUBE_PLACEMENT_ANGLE, false))
        .andThen(new AutoOuttake())
        .andThen(new DriveAlongPath("Red1ToPiece2"))
        .andThen(new Stop(0.2))
        .andThen(new PositionWithTarget(RED_GAME_PIECE_AUTO_LOCATIONS[2].toTranslation2d(), AnglesAndDistances.FLOOR_CUBE.getFirst(), AnglesAndDistances.FLOOR_CUBE.getSecond().plus(CUBE_OVERSHOOT), FLOOR_CUBE_PICKUP_ANGLE, false))
        .andThen(new AutoIntake())
        .andThen(new DriveAlongPath("Piece2ToRed1"))
        .andThen(new Stop(0.2))
        .andThen(new PositionWithTarget(RED_FLOOR_PLACEMENT_LOCATIONS[7].toTranslation2d(), AnglesAndDistances.FLOOR_CUBE.getFirst(), AnglesAndDistances.FLOOR_CUBE.getSecond(), FLOOR_CUBE_PLACEMENT_ANGLE, false))
        .andThen(new AutoOuttake()));
      autonomousChooser.addOption("Blue Left 2 Piece",
        new PositionWithTarget(BLUE_CUBE_PLACEMENT_LOCATIONS[6].toTranslation2d(), AnglesAndDistances.HIGH_CUBE.getFirst(), AnglesAndDistances.HIGH_CUBE.getSecond().plus(CUBE_OVERSHOOT), HIGH_CUBE_PLACEMENT_ANGLE, true)
        .andThen(new AutoOuttake())
        .andThen(new DriveAlongPath("Blue1ToPiece1"))
        .andThen(new Stop(0.1))
        .andThen(new PositionWithTarget(BLUE_GAME_PIECE_AUTO_LOCATIONS[3].toTranslation2d(), AnglesAndDistances.FLOOR_CUBE.getFirst(), AnglesAndDistances.FLOOR_CUBE.getSecond().plus(CUBE_OVERSHOOT), FLOOR_CUBE_PICKUP_ANGLE, false))
        .andThen(new AutoIntake())
        .andThen(new DriveAlongPath("Piece1ToBlue1"))
        .andThen(new Stop(0.1))
        .andThen(new PositionWithTarget(BLUE_CUBE_PLACEMENT_LOCATIONS[5].toTranslation2d(), AnglesAndDistances.MEDIUM_CUBE.getFirst(), AnglesAndDistances.MEDIUM_CUBE.getSecond(), MID_CUBE_PLACEMENT_ANGLE, false))
        .andThen(new AutoOuttake())
        .andThen(new DriveAlongPath("Blue1ToPiece2"))
        .andThen(new Stop(0.1))
        .andThen(new PositionWithTarget(BLUE_GAME_PIECE_AUTO_LOCATIONS[2].toTranslation2d(), AnglesAndDistances.FLOOR_CUBE.getFirst(), AnglesAndDistances.FLOOR_CUBE.getSecond().plus(CUBE_OVERSHOOT), FLOOR_CUBE_PICKUP_ANGLE, false))
        .andThen(new AutoIntake())
        .andThen(new DriveAlongPath("Piece2ToBlue1"))
        .andThen(new Stop(0.1))
        .andThen(new PositionWithTarget(BLUE_FLOOR_PLACEMENT_LOCATIONS[7].toTranslation2d(), AnglesAndDistances.FLOOR_CUBE.getFirst(), AnglesAndDistances.FLOOR_CUBE.getSecond(), FLOOR_CUBE_PLACEMENT_ANGLE, false))
        .andThen(new AutoOuttake()));
      autonomousChooser.addOption("Blue Right 2 Piece", new DriveAlongPath("Blue3ToPiece4")
        .andThen(new Stop(0.1))
        .andThen(new DriveAlongPath("Piece4ToBlue3"))
        .andThen(new Stop(0.1))
        .andThen(new DriveAlongPath("Blue3ToPiece3"))
        .andThen(new Stop(0.1))
        .andThen(new DriveAlongPath("Piece3ToBlue3"))
        .andThen(new Stop(0.1)));
      autonomousChooser.addOption("Red Left 2 Piece", new DriveAlongPath("Red3ToPiece4")
        .andThen(new Stop(0.1))
        .andThen(new DriveAlongPath("Piece4ToRed3"))
        .andThen(new Stop(0.1))
        .andThen(new DriveAlongPath("Red3ToPiece3"))
        .andThen(new Stop(0.1))
        .andThen(new DriveAlongPath("Piece3ToRed3"))
        .andThen(new Stop(0.1)));
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
    led.setDefaultCommand(new LEDInit());
  }
  public void periodic() {
  }
}

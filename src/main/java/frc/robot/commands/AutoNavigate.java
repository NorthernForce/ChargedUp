package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.coprocessors.Coprocessor;
import frc.lib.coprocessors.ros.RosNavigation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.ArmConstants;

import static frc.robot.RobotContainer.*;

public class AutoNavigate extends CommandBase {
    private RosNavigation.RosNavigationCommand navigationCommand = null;
    private SetArmAngle armCommand = null;
    private SetWristAngle wristCommand = null;
    private ExtendArm extendArm = null;
    private final NetworkTableEntry rowEntry, colEntry;
    public AutoNavigate()
    {
        rowEntry = NetworkTableInstance.getDefault().getTable("pieceSelector").getEntry("row");
        colEntry = NetworkTableInstance.getDefault().getTable("pieceSelector").getEntry("col");
    }
    @Override
    public void initialize()
    {
        if (navigation.getCoprocessorStatus() != Coprocessor.Status.ONLINE) return;
        var alliance = DriverStation.getAlliance();
        if (alliance == DriverStation.Alliance.Red)
        {
            if (navigation.getPose2d().getY() >= 8.2423)
            {
                int row = (int)rowEntry.getInteger(0);
                int col = (int)colEntry.getInteger(0);
                Translation3d targetPosition;
                if (row == 0)
                {
                    targetPosition = FieldConstants.RED_FLOOR_PLACEMENT_LOCATIONS[col];
                }
                else
                {
                    if ((col % 2) == 0)
                    {
                        targetPosition = FieldConstants.RED_CONE_PLACEMENT_LOCATIONS[col + row - 1];
                    }
                    else
                    {
                        targetPosition = FieldConstants.RED_CUBE_PLACEMENT_LOCATIONS[col + row - 2];
                    }
                }
                double length = (row > 1) ? ArmConstants.EXTENDED_LENGTH : ArmConstants.RETRACTED_LENGTH;
                double wristFulcrumToEnd = ((col % 2) == 0) ? Constants.GripperConstants.FULCRUM_TO_CONE :
                    Constants.GripperConstants.FULCRUM_TO_CUBE;
                Rotation2d wristAngle;
                if (row == 0)
                {
                    wristAngle = ((col % 2) == 0) ? Constants.WristConstants.LOW_CONE_PLACEMENT_ANGLE :
                        Constants.WristConstants.LOW_CUBE_PLACEMENT_ANGLE;
                }
                else if (row == 1)
                {
                    wristAngle = ((col % 2) == 0) ? Constants.WristConstants.MID_CONE_PLACEMENT_ANGLE :
                        Constants.WristConstants.MID_CUBE_PLACEMENT_ANGLE;
                }
                else
                {
                    wristAngle = ((col % 2) == 0) ? Constants.WristConstants.HIGH_CONE_PLACEMENT_ANGLE :
                        Constants.WristConstants.HIGH_CUBE_PLACEMENT_ANGLE;
                }
                var armSolved = Constants.calculateArmAngleAndDistance(
                    ArmConstants.ORIGIN.getZ(), length, wristFulcrumToEnd, wristAngle, targetPosition.getZ(),
                        ArmConstants.ORIGIN.getX(), Rotation2d.fromDegrees(15));
                navigationCommand = navigation.navigateTo(
                    new Pose2d(
                        targetPosition
                            .toTranslation2d()
                            .minus(new Translation2d(armSolved.getFirst(), 0)),
                        Rotation2d.fromDegrees(0)
                    )
                );
                armCommand = new SetArmAngle(armSolved.getSecond());
                wristCommand = new SetWristAngle(wristAngle);
                if (row > 1) extendArm = new ExtendArm();
            }
            else
            {
                navigationCommand = navigation.navigateTo(Constants.RED_SINGLE_SUBSTATION);
                armCommand = new SetArmAngle(Rotation2d.fromDegrees(160));
                wristCommand = new SetWristAngle(Rotation2d.fromDegrees(60));
            }
        }
        else
        {
            if (navigation.getPose2d().getY() < 8.2423)
            {
                int row = (int)rowEntry.getInteger(0);
                int col = (int)colEntry.getInteger(0);
                Translation3d targetPosition;
                if (row == 0)
                {
                    targetPosition = FieldConstants.BLUE_FLOOR_PLACEMENT_LOCATIONS[col];
                }
                else
                {
                    if ((col % 2) == 0)
                    {
                        targetPosition = FieldConstants.BLUE_CONE_PLACEMENT_LOCATIONS[col + row - 1];
                    }
                    else
                    {
                        targetPosition = FieldConstants.BLUE_CUBE_PLACEMENT_LOCATIONS[col + row - 2];
                    }
                }
                double length = (row > 1) ? ArmConstants.EXTENDED_LENGTH : ArmConstants.RETRACTED_LENGTH;
                double wristFulcrumToEnd = ((col % 2) == 0) ? Constants.GripperConstants.FULCRUM_TO_CONE :
                    Constants.GripperConstants.FULCRUM_TO_CUBE;
                Rotation2d wristAngle;
                if (row == 0)
                {
                    wristAngle = ((col % 2) == 0) ? Constants.WristConstants.LOW_CONE_PLACEMENT_ANGLE :
                        Constants.WristConstants.LOW_CUBE_PLACEMENT_ANGLE;
                }
                else if (row == 1)
                {
                    wristAngle = ((col % 2) == 0) ? Constants.WristConstants.MID_CONE_PLACEMENT_ANGLE :
                        Constants.WristConstants.MID_CUBE_PLACEMENT_ANGLE;
                }
                else
                {
                    wristAngle = ((col % 2) == 0) ? Constants.WristConstants.HIGH_CONE_PLACEMENT_ANGLE :
                        Constants.WristConstants.HIGH_CUBE_PLACEMENT_ANGLE;
                }
                var armSolved = Constants.calculateArmAngleAndDistance(
                    ArmConstants.ORIGIN.getZ(), length, wristFulcrumToEnd, wristAngle, targetPosition.getZ(),
                        ArmConstants.ORIGIN.getX(), Rotation2d.fromDegrees(15));
                navigationCommand = navigation.navigateTo(
                    new Pose2d(
                        targetPosition
                            .toTranslation2d()
                            .plus(new Translation2d(armSolved.getFirst(), 0)),
                        Rotation2d.fromDegrees(180)
                    )
                );
                armCommand = new SetArmAngle(armSolved.getSecond());
                wristCommand = new SetWristAngle(wristAngle);
                if (row > 1) extendArm = new ExtendArm();
            }
            else
            {
                navigationCommand = navigation.navigateTo(Constants.BLUE_SINGLE_SUBSTATION);
                armCommand = new SetArmAngle(Rotation2d.fromDegrees(160));
                wristCommand = new SetWristAngle(Rotation2d.fromDegrees(60));
            }
        }
        armCommand.schedule();
        wristCommand.schedule();
        navigationCommand.schedule();
        if (extendArm != null) extendArm.schedule();
    }
    @Override
    public boolean isFinished()
    {
        if (armCommand != null && !armCommand.isFinished()) return false;
        if (navigationCommand != null && !navigationCommand.isFinished()) return false;
        if (wristCommand != null && !wristCommand.isFinished()) return false;
        if (extendArm != null && !extendArm.isFinished()) return false;
        return true;
    }
    @Override
    public void end(boolean interrupted)
    {
        if (armCommand != null) armCommand.cancel();
        if (navigationCommand != null) navigationCommand.cancel();
        if (wristCommand != null) wristCommand.cancel();
        if (extendArm != null) extendArm.cancel();
        armCommand = null;
        navigationCommand = null;
        wristCommand = null;
        extendArm = null;
    }
}

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.cameras.PhotonCameraWrapper;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.OI;

import static frc.robot.RobotContainer.*;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

public class AutoAlign extends CommandBase {
    private DoubleSupplier[] driverSpeeds = OI.getDriveSuppliers();
    private PhotonCameraWrapper forward = activeChassis.getPhotonCameras().get(0);
    private Rotation2d startAngle;
    public enum Stage
    {
        NOT_AT_TURN,
        TURNING,
        BRAKING
    }
    private Stage stage;
    public AutoAlign()
    {
        addRequirements(drivetrain);
    }
    @Override
    public void initialize()
    {
        stage = Stage.NOT_AT_TURN;
    }
    @Override
    public void execute()
    {
        if (stage == Stage.NOT_AT_TURN)
        {
            if (forward.hasTarget() && forward.getFiducialId() == (DriverStation.getAlliance() == Alliance.Red ? 4 : 8))
            {
                double apriltagDistance = forward.getTargetDistance(FieldConstants.APRILTAG_LAYOUT.getTagPose(4).get().getY());
                if (apriltagDistance <= Constants.AnglesAndDistances.APRILTAG_DIST)
                {
                    stage = Stage.TURNING;
                    startAngle = imu.getRotation2d();
                    return;
                }
            }
            drivetrain.drive(driverSpeeds[0].getAsDouble(), driverSpeeds[1].getAsDouble());
        }
        else if (stage == Stage.TURNING)
        {
            if (DriverStation.getAlliance() == Alliance.Red)
            {
                if (imu.getRotation2d().minus(startAngle).getDegrees() > 90)
                {
                    stage = Stage.BRAKING;
                    return;
                }
            }
            else
            {
                if (startAngle.minus(imu.getRotation2d()).getDegrees() > 90)
                {
                    stage = Stage.BRAKING;
                    return;
                }
            }
            drivetrain.drive(0, (DriverStation.getAlliance() == Alliance.Red ? 1 : -1));
        }
        else
        {
            drivetrain.drive(0, 0);
        }
    }
}

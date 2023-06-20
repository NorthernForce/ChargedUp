package frc.robot.commands.autoComponents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.Constants.DrivetrainConstants;

public class AutoAlign extends DriveToLocation {
    private Alliance alliance;
    private final Pose2d bluePose;
    public AutoAlign(Pose2d bluePose)
    {
        super(bluePose, DrivetrainConstants.ALIGN_MAX_VEL, DrivetrainConstants.ALIGN_MAX_ACCEL);
        this.bluePose = bluePose;
    }
    @Override
    public void initialize()
    {
        alliance = DriverStation.getAlliance();
        if (alliance == Alliance.Red)
        {
            targetPose = bluePose.relativeTo(new Pose2d(
                new Translation2d(
                    16.4846,
                    0
                ),
                Rotation2d.fromDegrees(180)
            ));
        }
        super.initialize();
    }
}

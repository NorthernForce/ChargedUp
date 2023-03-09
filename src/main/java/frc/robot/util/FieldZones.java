// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import javax.swing.plaf.basic.BasicBorders.FieldBorder;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.Constants.APRILTAG_LAYOUT;

/** Add your docs here. */
public class FieldZones {
    enum Locations {
        GRIDS,
        OUTER_LOAD,
        INNER_LOAD;
        /**
         * Gets the coordinates that the zone is defined by
         * @return Two Pose2d. [0] = lower corner. [1] = upper corner.
         */
        Pose2d[] getLocationRestraints() {
            Pose2d[] coords;
            switch (this) {
                case GRIDS:
                    coords = new Pose2d[]{
                        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), null),
                        new Pose2d(Units.inchesToMeters(110),  Units.inchesToMeters(208), null)
                    };
                    if (DriverStation.getAlliance() == Alliance.Red) {
                        return allianceFlip(coords);
                    }
                    return coords;
                case OUTER_LOAD:
                    coords = new Pose2d[]{
                        new Pose2d(),
                        new Pose2d(Units.inchesToMeters(640),0, null)
                    };
                    if (DriverStation.getAlliance () == Alliance.Red) {
                        return allianceFlip(coords);
                    }
                    return coords;
                case INNER_LOAD:
                    coords = new Pose2d[]{
                        new Pose2d(),
                        new Pose2d()
                    };
                    if (DriverStation.getAlliance() == Alliance.Red) {
                        return allianceFlip(coords);
                    }
                    return coords;
                default:
                    return null;
            }
        }

        boolean checkLocation() {
            return false;
        }
    }
    /**
     * Takes a red/blue Pos and makes it the opposite one. Only flips across center line
     * @param originPos This is the pose2ds to be converted
     * @return Returns the pose2d for the other side of field
     */
    private static Pose2d[] allianceFlip(Pose2d[] originPos) {
        for (int i=0; i < originPos.length; i++) {
            double FIELD_LENGTH = 16.4846; //METERS
            double newX = FIELD_LENGTH - originPos[i].getX();
            originPos[i] = new Pose2d(newX, originPos[i].getY(), originPos[i].getRotation());
        }
        return originPos;
    }
}

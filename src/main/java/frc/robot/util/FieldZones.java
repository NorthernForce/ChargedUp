// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldZones {
    public enum Zone {
        GRIDS,
        OUTER_LOAD,
        INNER_LOAD,
        SLIDE_DROP_LOAD,
        NONE;
        /**
         * Gets the coordinates that the zone is defined by
         * @return Two Pose2d. [0] = lower corner. [1] = upper corner.
         */
        Pose2d[] getZoneRestraints() {
            Pose2d[] coords;
            switch (this) {
                case GRIDS:
                    coords = new Pose2d[]{
                        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), null),
                        new Pose2d(Units.inchesToMeters(110),  Units.inchesToMeters(208), null)
                    };
                    return setAlliance(DriverStation.getAlliance(), coords);
                case OUTER_LOAD:
                    coords = new Pose2d[]{
                        new Pose2d(Units.inchesToMeters(500), Units.inchesToMeters(267), null),
                        new Pose2d(Units.inchesToMeters(645),Units.inchesToMeters(323), null)
                    };
                    return setAlliance(DriverStation.getAlliance(), coords);
                case INNER_LOAD:
                    coords = new Pose2d[]{
                        new Pose2d(Units.inchesToMeters(500), Units.inchesToMeters(213), null),
                        new Pose2d(Units.inchesToMeters(645),Units.inchesToMeters(267), null)
                    };
                case SLIDE_DROP_LOAD:
                    coords = new Pose2d[]{
                        /*
                         * x bottom = slide xLeft at 548", slidewidth = 25", so midpoint = 560.5". Then, other zones are 145" wide, so found 560.5" - 72.5".
                         * y bottom = slide y at 317". Other zones are 54" and 56" long. So 317 - 54" is 263".
                         * 
                         * x top    = as above. midpoint 560.5" + 72.5" = 633"
                         * y top    = as above, slide y at 317".
                         * 
                         * TODO: do these zones overlap. Focusing on slide for now so not bothering to check.
                         */
                        new Pose2d(Units.inchesToMeters(488), Units.inchesToMeters(263), null), 
                        new Pose2d(Units.inchesToMeters(633), Units.inchesToMeters(317), null) // 
                    };
                    return setAlliance(DriverStation.getAlliance(), coords);
                default:
                    return null; // TODO: Don't return null. Or. Who calls this, what do they want it to return.
            }
        }
        /**
         * Used to check if the robot is within the given field zone location.
         * @param current the pose2d to be compared to current zone
         * @return boolen representing (robot in zone)
         */
        boolean inZone(Pose2d current) {
            Pose2d[] restraints = this.getZoneRestraints();
            //If the robots current x position is outside of the restraints it returns false.
            if (!(restraints[0].getX() < current.getX() && current.getX() < restraints[1].getX())) {
                return false;
            }
            //If the robots current y position is outside of the restraints it returns false
            if (!(restraints[0].getY() < current.getY() && current.getY() < restraints[1].getY())) {
                return false;
            }
            return true;
        }
    }
    
    /**
     * Checks the zone of a passed in pose2d
     * @param current
     * @return
     */
    public Zone getCurrentZone(Pose2d current) {
        for (Zone zone : Zone.values()) {
            if (zone.inZone(current)) {
                return zone;
            }
        }
        return Zone.NONE;
    }
    /**
     * Takes a blue Pos and makes it the opposite one. Only flips across center line
     * @param bluePoses This is the pose2ds of blue alliance to be calculated on.
     * @return Returns the pose2d for the other side of field
     */
    public static Pose2d[] setAlliance(Alliance alliance, Pose2d[] bluePoses) {
        if ( alliance == Alliance.Blue )
        {
            return bluePoses;
        }
        else 
        {
            for (int i=0; i < bluePoses.length; i++) {
                double FIELD_LENGTH = 16.4846; //METERS
                double newX = FIELD_LENGTH - bluePoses[i].getX();
                bluePoses[i] = new Pose2d(newX, bluePoses[i].getY(), bluePoses[i].getRotation());
            }
            return bluePoses;
        }
    }
}

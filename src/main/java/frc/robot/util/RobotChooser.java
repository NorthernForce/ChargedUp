// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import frc.robot.chassis.*;

/** This is RobotChooser. It reads the persistent ID on the roboRIO to choose a chassis.*/
public class RobotChooser 
{
    private static final String ID_FILE_PATH = "/home/admin/robot_settings.txt";
    private static final String SQUISHY_ID = "Squishy";
    private static final String SPEEDY_ID = "Speedy";
    private static final String MAXWELL_ID = "Maxwell";
    /**
     * Returns the persisten ID stored on the robot
     * @return String value from robot  
     */
    private static String readRobotID() 
    {
        try 
        {
            File myObj = new File(ID_FILE_PATH);
            Scanner myReader = new Scanner(myObj);
            String str = myReader.nextLine();
            myReader.close();
            return str;
        } catch (FileNotFoundException e)
        {
            System.err.println("Could not open file " + ID_FILE_PATH);
            e.printStackTrace();
            return "";
        }

    }
    /**
     * Chooses drivetrain object based on persistent ID on robot.
     * @return Drivetrain object for current robot.
     */
    public static ChassisBase getChassis()
    {
        ChassisBase chassis;
        String id = readRobotID();
        switch (id) {
            case SQUISHY_ID:
                chassis = new SquishyChassis();
                break;
            case SPEEDY_ID:
                chassis = new SpeedyChassis();
                break;
            case MAXWELL_ID:
                chassis = new MaxwellChassis();
                break;
            default:
                chassis = new MaxwellChassis();
                break;
        }
        return chassis;
    }
}
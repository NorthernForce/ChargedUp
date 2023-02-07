// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import frc.robot.util.robots.*;
import frc.robot.subsystems.DrivetrainSuper;
import frc.robot.subsystems.DrivetrainSpeedy;
import frc.robot.subsystems.DrivetrainSquishy;

/** Add your docs here. */
public class RobotChooser 
{
    private final String ID_FILE_PATH = "filename.txt";
    private final String SQUISHY_ID = "squishy";
    private final String SPEEDY_ID = "speedy";
    public RobotChooser() {}

    /**
     * Returns the persisten ID stored on the robot
     * @return String value from robot  
     */
    private String ReadRobotID() 
    {
        try 
        {
            File myObj = new File(ID_FILE_PATH);
            Scanner myReader = new Scanner(myObj);
            myReader.close();
            return myReader.nextLine();
        } catch (FileNotFoundException e)
        {
            System.out.println("An error occured.");
            e.printStackTrace();
            return "";
        }

    }

    /**
     * Creates a robot object when given a valid ID.
     * @param id String identifying the robot
     */
    private RobotSuper CreateObject(String id)
    {
        RobotSuper tempBot;
        switch (id) {
            case SQUISHY_ID:
                tempBot = new Squishy();
                break;
            case SPEEDY_ID:
                tempBot = new Speedy();
                break;
            default:
                tempBot = new Speedy();
                break;
        }
        return tempBot;
    }

    /**
     * Chooses drivetrain object based on persistent ID on robot.
     * @return Drivetrain object for current robot.
     */
    public DrivetrainSuper GetDrivetrain()
    {
        DrivetrainSuper drivetrain;
        String id = ReadRobotID();
        switch (id) {
            case SQUISHY_ID:
                drivetrain = new DrivetrainSquishy();
                break;
            case SPEEDY_ID:
                drivetrain = new DrivetrainSpeedy();
                break;
            default:
                drivetrain = new DrivetrainSpeedy();
                break;
        }
        return drivetrain;
    }

    /**
     * Returns a robot object corresponding to the robot running the code.
     * @return Robot object for current robot.
     */
    public RobotSuper SelectRobot()
    {
        String id = ReadRobotID();
        return CreateObject(id);
    }
}
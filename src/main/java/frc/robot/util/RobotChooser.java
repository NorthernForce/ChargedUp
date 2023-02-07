// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.variants.DrivetrainSpeedy;
import frc.robot.subsystems.variants.DrivetrainSquishy;

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
     * Chooses drivetrain object based on persistent ID on robot.
     * @return Drivetrain object for current robot.
     */
    public Drivetrain GetDrivetrain()
    {
        Drivetrain drivetrain;
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
}
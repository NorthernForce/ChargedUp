// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.lang.reflect.Field;

import frc.robot.Constants;
import frc.robot.chassis.*;
import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/** This is RobotChooser. It reads the persistent ID on the roboRIO to choose a chassis.*/
public class RobotChooser 
{
    private final String ID_FILE_PATH = "/home/admin/robot_settings.json";
    private final String SQUISHY_ID = "Squishy";
    private final String SPEEDY_ID = "Speedy";
    private final String MAXWELL_ID = "Maxwell";
    public RobotChooser() {}

    /**
     * Returns the persisten ID stored on the robot
     * @return String value from robot  
     * @throws IllegalAccessException
     * @throws IllegalArgumentException
     */
    private String readRobotID()
    {
        try 
        {
            File file = new File(ID_FILE_PATH);
            ObjectMapper mapper = new ObjectMapper();
            JsonNode mainNode = mapper.readTree(file);
            String name = mainNode.get("RobotName").asText("Maxwell");
            var constantsClass = Constants.class;
            var iterator = mainNode.fields();
            while (iterator.hasNext())
            {
                var constant = iterator.next();
                Field field = constantsClass.getField(constant.getKey());
                if (constant.getValue().isDouble())
                {
                    field.setDouble(null, constant.getValue().asDouble());
                }
                else if (constant.getValue().isInt())
                {
                    field.setInt(null, constant.getValue().asInt());
                }
                else if (constant.getValue().isBoolean())
                {
                    field.setBoolean(null, constant.getValue().asBoolean());
                }
                else if (constant.getValue().isTextual())
                {
                    field.set(null, constant.getValue().textValue());
                }
            }
            return name;
        } catch (FileNotFoundException e)
        {
            System.err.println("Could not open file " + ID_FILE_PATH);
            e.printStackTrace();
        } catch (JsonParseException e) {
            System.err.println("Error parsing json file " + ID_FILE_PATH);
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NoSuchFieldException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (SecurityException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IllegalArgumentException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return "";
    }

    /**
     * Chooses drivetrain object based on persistent ID on robot.
     * @return Drivetrain object for current robot.
     */
    public ChassisBase getChassis()
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
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.states.manipulatingstate;

/** Add your docs here. */
public class ManipulatingStateContainer {
    ConeManipulatingState coneManipulatingState;
    CubeManipulatingState cubeManipulatingState;
    EmptyManipulatingState emptyManipulatingState;
    ManipulatingState currentState;

    public ManipulatingStateContainer() {
        coneManipulatingState = new ConeManipulatingState();
        cubeManipulatingState = new CubeManipulatingState();
        emptyManipulatingState = new EmptyManipulatingState();
        currentState = emptyManipulatingState;
    }

    /**
     * Used to access the object for the current state
     * @return the current manipulatingState object
     */
    public ManipulatingState getCurrentState() {
        return currentState;
    }
    
    /** Sets the current state to cones */
    public void setCone() {
        currentState = coneManipulatingState;
    }

    /** Sets the current state to cubes */
    public void setCube() {
        currentState = cubeManipulatingState;
    }

    /** Sets the current state to empty */
    public void emptyState() {
        currentState = emptyManipulatingState;
    }
}

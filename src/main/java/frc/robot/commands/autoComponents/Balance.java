// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoComponents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import static frc.robot.RobotContainer.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Balance extends PIDCommand {
  /** Creates a new Balance. */
  private final Timer timer = new Timer();

  public Balance() {
    super(
        // The controller that the command will use
        new PIDController(0.038, 0, 0),
        // This should return the measurement
        () -> imu.getPitch().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drivetrain.drive(-output, 0);
          SmartDashboard.putNumber("Forward Speed", output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  @Override 
  public void end(boolean interrupted) {
    if (!interrupted) {
      
      

    }

  }
  @Override
  public void execute()
  {
    super.execute();
    if (!getController().atSetpoint())
    {
      timer.reset();
    }
    if (timer.hasElapsed(0.2))
    {
      led.rainbow();
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished()
   {
    return false;

  }
}

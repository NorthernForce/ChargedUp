package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.led;

public class LEDAdjustment extends CommandBase {
    private final double seconds;
    private double startTime;
/** Creates a new LEDAdjustment. */
public LEDAdjustment(double seconds) {
    this.seconds = seconds;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     this.startTime = Timer.getFPGATimestamp();
    }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.rainbow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setPink();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double nowSeconds = Timer.getFPGATimestamp();
    return ((nowSeconds - startTime) >= seconds);
    
  }
}

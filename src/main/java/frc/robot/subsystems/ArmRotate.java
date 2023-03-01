// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.variants.MotorGroupTalon;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

public class ArmRotate extends ProfiledPIDSubsystem {
  // We know we will have two talons
  private final MotorGroupTalon talonGroup;
  private final CANCoder rotateEncoder;
  private final CANCoderSimCollection rotateEncoderSim;
  private final ArmFeedforward feedforward = new ArmFeedforward(Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV);
  private final SingleJointedArmSim armSim;
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d armMech =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              90,
              6,
              new Color8Bit(Color.kYellow)));
  /** Creates a new Arm. */
  public ArmRotate() {
    super(
      new ProfiledPIDController(
        Constants.ARM_PROPORTION,
        0,
        0,
        new Constraints(Constants.ARM_MAX_RADIANS_PER_SECOND, Constants.ARM_MAX_RADIANS_PER_SECOND_SQUARED)
      )
    );
    talonGroup = new MotorGroupTalon(Constants.ARM_PRIMARY_MOTOR_ID, new int[]
    {
      Constants.ARM_FOLLOWER_MOTOR_ID
    });
    talonGroup.setFollowerOppose(0);
    rotateEncoder = new CANCoder(Constants.ARM_ROTATE_CANCODER_ID);
    if (RobotBase.isSimulation())
    {
      rotateEncoderSim = rotateEncoder.getSimCollection();
      armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(2),
        2200.0 / 634,
        SingleJointedArmSim.estimateMOI(Constants.ARM_RETRACTED_LENGTH, 9.07185),
        Constants.ARM_RETRACTED_LENGTH,
        -Math.toRadians(-30),
        Math.toRadians(210),
        true
      );
      System.out.println("????????? " + armSim.getAngleRads());
    }
    else
    {
      armSim = null;
      rotateEncoderSim = null;
    }
    Shuffleboard.getTab("Arm").add("Arm diagram", mech2d);
    armTower.setColor(new Color8Bit(Color.kBlue));
    setAngle(Rotation2d.fromDegrees(-20));
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(rotateEncoder.getPosition());
  }
  /**
   * Set arm angle
   * @param angle angle to set the arm to
  */
  public void setAngle(Rotation2d angle)
  {
    setGoal(angle.getRadians());
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmSpeed(double speed)
  {
    setGoal(getController().getGoal().position + Rotation2d.fromDegrees(speed).getRadians());
  }
  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Arm Angle", getAngle().getDegrees());
    armMech.setAngle(getAngle());
  }
  @Override
  protected void useOutput(double output, State setpoint) {
    System.out.println(output + ": " + setpoint);
    talonGroup.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
  }
  @Override
  protected double getMeasurement() {
    return getAngle().getRadians();
  }
  @Override
  public void simulationPeriodic()
  {
    talonGroup.setSimulationBusVoltage(RobotController.getBatteryVoltage());
    System.out.println(talonGroup.getSimulationOutputVoltage());
    armSim.setInputVoltage(talonGroup.getSimulationOutputVoltage());
    armSim.update(0.02);
    rotateEncoderSim.setRawPosition((int)(4096 * Rotation2d.fromRadians(armSim.getAngleRads()).getRotations()));
  }
}
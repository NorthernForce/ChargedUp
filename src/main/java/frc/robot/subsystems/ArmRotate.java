// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.variants.MotorGroupTalon;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

public class ArmRotate extends SubsystemBase {
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
              0,
              6,
              new Color8Bit(Color.kYellow)));
  /** Creates a new Arm. */
  public ArmRotate() {
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
        Math.toRadians(-150),
        Math.toRadians(70),
        true
      );
      armSim.setState(new MatBuilder<N2, N1>(N2.instance, N1.instance).fill(90, 0));
      rotateEncoderSim.setRawPosition((int)(4096 * Rotation2d.fromRadians(armSim.getAngleRads()).getRotations()));
    }
    else
    {
      armSim = null;
      rotateEncoderSim = null;
    }
    Shuffleboard.getTab("Arm").add("Arm diagram", mech2d);
    Shuffleboard.getTab("Arm").addNumber("Arm Angle", () -> getAngle().getDegrees());
    armTower.setColor(new Color8Bit(Color.kBlue));
  }
  /**
   * Get arm angle
   * @return Offset from horizon line
   */
  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(rotateEncoder.getPosition());
  }
  public void setSimAngle(Rotation2d angle)
  {
    armSim.setState(new MatBuilder<N2, N1>(N2.instance, N1.instance).fill(angle.getDegrees(), 0));
    rotateEncoderSim.setRawPosition((int)(4096 * Rotation2d.fromRadians(armSim.getAngleRads()).getRotations()));
  }
  /**
   * Set arm angular speed
   * @param speed between 1.0 and -1.0
   */
  public void setArmVoltage(double speed)
  {
    talonGroup.setVoltage(speed);
  }
  public ArmFeedforward getFeedforward()
  {
    return feedforward;
  }
  @Override
  public void periodic() {
    super.periodic();
    armMech.setAngle(getAngle());
    
  }
  @Override
  public void simulationPeriodic()
  {
    talonGroup.setSimulationBusVoltage(RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("talonGroup.get", talonGroup.getSimulationOutputVoltage());
    armSim.setInputVoltage(talonGroup.getSimulationOutputVoltage());
    armSim.update(0.02);
    SmartDashboard.putNumber("Arm Sim Angle Rads.", Math.toDegrees(armSim.getAngleRads()));
    rotateEncoderSim.setRawPosition((int)(4096 * Rotation2d.fromRadians(armSim.getAngleRads()).getRotations()));
  }
}
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IMU extends SubsystemBase {
  private final AHRS ahrs = new AHRS();
  /** Creates a new IMU. */
  public IMU() {
  }
  /**
   * Gets the pitch. Roll and Pitch are swapped on specific NavXs
   * @return pitch in Rotation2d
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(Constants.IMUConstants.ROLL_PITCH_SWAPPED ? ahrs.getRoll() : ahrs.getPitch());
  }
  /**
   * Gets the yaw.
   * @return yaw in Rotation2d
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(ahrs.getYaw());
  }
  /**
   * Gets the roll. Roll and Pitch are swapped on specific NavXs
   * @return roll in Rotation2d
   */
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(Constants.IMUConstants.ROLL_PITCH_SWAPPED ? ahrs.getPitch() : ahrs.getRoll());
  }
  /**
   * Gets the current Rotation2d
   * @return Rotation2d of Yaw
   */
  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d();
  }
  /** Calibrates the IMU */
  public void calibrate() {
    ahrs.calibrate();
  }
  /**
   * Tells whether the IMU is calibrating
   * @return whether the IMU is calibrating
   */
  public boolean isCalibrating() {
    return ahrs.isCalibrating();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
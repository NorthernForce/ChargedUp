package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.Map;

/**
 * Represents a single point in two-dimensional space.
 */
public final class PieceLocation implements Sendable {

  private double x;
  private double y;

  /**
   * Creates a new 2D point object.
   *
   * @param x the X-coordinate of the point
   * @param y the Y-coordinate of the point
   */
  public PieceLocation(double x, double y) {
    this.x = x;
    this.y = y;
  }

  /**
   * Gets the X-coordinate of this point.
   */
  public double getX() {
    return x;
  }

  /**
   * Gets the Y-coordinate of this point.
   */
  public double getY() {
    return y;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    builder.setSmartDashboardType("PieceLocation");
    builder.addDoubleProperty("x", () -> getX(), x -> this.x = x);
    builder.addDoubleProperty("y", () -> getY(), y -> this.y = y);
  }
}

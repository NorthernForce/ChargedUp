package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Point2D implements Sendable {
    private double x;
    private double y;
    public Point2D(double x, double y)
    {
        this.x = x;
        this.y = y;
        
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("x", this::getX, this::setX);
        builder.addDoubleProperty("y", this::getY, this::setY);
    }
    public double getX()
    {
        return x;
    }
    public double getY()
    {
        return y;
    }
    public void setX(double x)
    {
        this.x = x;
    }
    public void setY(double y)
    {
        this.y = y;
    }
}

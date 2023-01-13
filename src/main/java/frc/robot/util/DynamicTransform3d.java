package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.Objects;

public class DynamicTransform3d extends Transform3d {
    private Translation3d translation;
    private Rotation3d rotation;
    public DynamicTransform3d()
    {
        super();
        translation = new Translation3d();
        rotation = new Rotation3d();
    }
    public DynamicTransform3d(Pose3d initial, Pose3d last)
    {
        super();
        translation = last.getTranslation().minus(initial.getTranslation()).rotateBy(initial.getRotation().unaryMinus());
        rotation = last.getRotation().minus(initial.getRotation());
    }
    public DynamicTransform3d(Translation3d translation, Rotation3d rotation)
    {
        super();
        this.translation = translation;
        this.rotation = rotation;
    }
    @Override
    public Transform3d times(double scalar)
    {
        return new Transform3d(translation.times(scalar), rotation.times(scalar));
    }
    @Override
    public Transform3d div(double scalar)
    {
        return times(1.0 / scalar);
    }
    @Override
    public Translation3d getTranslation()
    {
        return translation;
    }
    @Override
    public double getX()
    {
        return translation.getX();
    }
    @Override
    public double getY()
    {
        return translation.getY();
    }
    @Override
    public double getZ()
    {
        return translation.getZ();
    }
    @Override
    public Rotation3d getRotation()
    {
        return rotation;
    }
    public void setRotation(Rotation3d rotation)
    {
        this.rotation = rotation;
    }
    public void setTranslation(Translation3d translation)
    {
        this.translation = translation;
    }
    @Override
    public String toString()
    {
        return String.format("DynamicTransform3d(%s, %s)", translation, rotation);
    }
    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof DynamicTransform3d)
        {
            var transform = (DynamicTransform3d)obj;
            return transform.translation.equals(translation) && transform.rotation.equals(rotation);
        }
        return false;
    }
    @Override
    public int hashCode()
    {
        return Objects.hash(translation, rotation);
    }
}
package org.frc5687.deepspace.robot.utils.math;

public class InterpolatingDouble implements Interpolable<InterpolatingDouble> {
    public double _value;

    public InterpolatingDouble(double val) {
        _value = val;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
        double differnece = other._value - _value;
        return new InterpolatingDouble(differnece * x + _value);
    }
}
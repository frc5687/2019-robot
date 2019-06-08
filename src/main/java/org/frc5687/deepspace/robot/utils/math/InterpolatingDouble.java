package org.frc5687.deepspace.robot.utils.math;

public class InterpolatingDouble implements Interpolable<InterpolatingDouble> {

    private double _value;

    public InterpolatingDouble(double value) {
        this._value = value;
    }

    public double getValue() {
        return _value;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double percentage) {
        double diff = other.getValue() - _value;
        return new InterpolatingDouble(_value + (diff * percentage));
    }
}

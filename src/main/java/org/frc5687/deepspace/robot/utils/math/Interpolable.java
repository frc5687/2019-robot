package org.frc5687.deepspace.robot.utils.math;

public interface Interpolable<T> {
    public T interpolate(T other, double x);
}

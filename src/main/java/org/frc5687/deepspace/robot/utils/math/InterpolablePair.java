package org.frc5687.deepspace.robot.utils.math;

public class InterpolablePair<T extends Interpolable<T>> {

    private T value;
    private double key;

    public InterpolablePair(double key, T value) {
        this.key = key;
        this.value = value;
    }

    public double getKey() {
        return key;
    }

    public T getValue() {
        return value;
    }
}

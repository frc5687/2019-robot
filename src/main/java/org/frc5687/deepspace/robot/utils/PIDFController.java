package org.frc5687.deepspace.robot.utils;

public class PIDFController {

    private double _error;
    private double _prevError;
    private double _integral;
    private double _tolerance;

    private double _maxOutput;
    private double _minOutput;

    private double _kP;
    private double _kI;
    private double _kD;
    private double _kF;

    private double _maxI;

    private final double _deltaTime = 0.02; //20 ms

    public PIDFController(double p, double i, double d, double f, double tolerance) {
        _kP = p;
        _kI = i;
        _kD = d;
        _kF = f;
        _tolerance = tolerance;

        _maxOutput = 1.0;
        _minOutput = -1.0;
        _prevError = 0.0;
        _error = 0.0;
        _integral = 0.0;
        _maxI = 50;
    }

    public double updatePIDF(double position, double setpoint) {
        _error = setpoint - position;
        double pValue;
        double iValue;
        double dValue;
        double feedForward;

        pValue = _kP * _error;
        // If we are over the max Integral error, then set integral to zero to account for Integral Windup.
        if (Math.abs(_error) < _maxI) {
            _integral += _error * _deltaTime;
        } else {
            _integral = 0.0;
        }
        iValue = _kI * _integral;

        dValue = _kD * (_error - _prevError)/_deltaTime;

        feedForward = _kF * setpoint;

        double output = pValue + iValue + dValue + feedForward;

        output = Helpers.limit(output, _minOutput, _maxOutput);

        _prevError = _error;

        if (Math.abs(_error) <= _tolerance) {
            return 0;
        } else {
            return output;
        }
    }

    public void setTolerance(double tolerance) {
        _tolerance = tolerance;
    }
    public double getError() { return _error; }

    public void setMaxI(double maxI) {
        _maxI = maxI;
    }

    public void setMinMaxOutput(double min, double max) {
        _minOutput = min;
        _maxOutput = max;
    }

    public void setConstants(double p, double i, double d, double f) {
        _kP = p;
        _kI = i;
        _kD = d;
        _kF = f;
    }

}

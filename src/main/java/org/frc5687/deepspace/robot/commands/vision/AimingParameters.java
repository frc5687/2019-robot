package org.frc5687.deepspace.robot.commands.vision;

import org.frc5687.deepspace.robot.utils.math.Rotation2D;

public class AimingParameters {
    private double _range;
    private double _lastSeenTimestamp;
    private double _stability;
    private Rotation2D _robotToGoal;

    public AimingParameters(double range, Rotation2D robotToGoal, double lastSeenTimestamp, double stability) {
        this._range = range;
        this._robotToGoal = robotToGoal;
        this._lastSeenTimestamp = lastSeenTimestamp;
        this._stability = stability;
    }

    public double getRange() {
        return _range;
    }

    public Rotation2D getRobotToGoal() {
        return _robotToGoal;
    }

    public double getLastSeenTimestamp() {
        return _lastSeenTimestamp;
    }

    public double getStability() {
        return _stability;
    }
}

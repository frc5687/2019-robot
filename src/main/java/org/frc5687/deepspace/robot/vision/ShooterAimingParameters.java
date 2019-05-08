package org.frc5687.deepspace.robot.vision;

import team254.lib.geometry.Rotation2d;

public class ShooterAimingParameters {
    double range;
    double lastSeenTarget;
    double stability;
    Rotation2d robotToGoal;
    Rotation2d targetOrientation;

    public ShooterAimingParameters(double range, Rotation2d robotToGoal, double lastSeenTarget, double stability, Rotation2d targetOrientation) {
        this.range = range;
        this.robotToGoal = robotToGoal;
        this.lastSeenTarget = lastSeenTarget;
        this.stability = stability;
        this.targetOrientation = targetOrientation;
    }

    public double getRange() {
        return range;
    }

    public double getLastSeenTarget() {
        return lastSeenTarget;
    }

    public double getStability() {
        return stability;
    }

    public Rotation2d getRobotToGoal() {
        return robotToGoal;
    }

    public Rotation2d getTargetOrientation() {
        return targetOrientation;
    }
}

package org.frc5687.deepspace.robot;

import org.frc5687.deepspace.robot.commands.vision.AimingParameters;
import org.frc5687.deepspace.robot.commands.vision.GoalTracker;
import org.frc5687.deepspace.robot.commands.vision.TargetInfo;
import org.frc5687.deepspace.robot.utils.Kinematics;
import org.frc5687.deepspace.robot.utils.math.*;


import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import static org.frc5687.deepspace.robot.Constants.Limelight.*;
import static org.frc5687.deepspace.robot.Constants.Vision.*;

public class RobotState {

    private static final int observationBufferSize = 100;

    private static final RigidTransform2D vehicleToCamera = new RigidTransform2D(
            new Translation2D(LIMELIGHT_X_OFFSET_FROM_CENTER, LIMELIGH_Y_OFFSET_FROM_CENTER), new Rotation2D());

    //FPGATimestamp -> RigidTransform2D or Rotation2D
    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2D> _fieldToVehicle;
    private Twist2D _vehicleVelocityPredicted;
    private Twist2D _vehicleVelocityMeasured;
    private double _distanceDriven;
    private GoalTracker _goalTracker;
    private Rotation2D _cameraPitchCorrection;
    private Rotation2D _cameraYawCorrection;
    private double _differentialHeight;
    private AimingParameters _cachedAimingParameters = null;


    private RobotState() {
        reset(0, new RigidTransform2D());
    }

    public synchronized void reset(double startTime, RigidTransform2D initialFieldToVehicle) {
        _fieldToVehicle = new InterpolatingTreeMap<>(observationBufferSize);
        _fieldToVehicle.put(new InterpolatingDouble(startTime), initialFieldToVehicle);
        _vehicleVelocityPredicted = Twist2D.getIdentity();
        _vehicleVelocityMeasured = Twist2D.getIdentity();
        _goalTracker = new GoalTracker();
        _cameraPitchCorrection = Rotation2D.fromDegrees(-LIMELIGHT_ANGLE);
        _cameraYawCorrection = Rotation2D.fromDegrees(-LIMELIGHT_YAW_ANGLE);
        _differentialHeight = TARGET_HEIGHT - LIMELIGHT_HEIGHT;
        _distanceDriven = 0.0;
    }
    public synchronized void resetDistanceDriven() {
        _distanceDriven = 0.0;
    }

    public synchronized RigidTransform2D getFieldToVehicle(double timestamp) {
        return _fieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2D> getLatestFieldToVehicle() {
        return _fieldToVehicle.lastEntry();
    }

    public synchronized RigidTransform2D getPredictedFieldToVehicle(double lookaheadTime) {
        return getLatestFieldToVehicle().getValue().transformBy(RigidTransform2D.exp(_vehicleVelocityPredicted.scaled(lookaheadTime)));
    }

    public synchronized RigidTransform2D getFieldToCamera(double timestamp){
        return getFieldToVehicle(timestamp).transformBy(vehicleToCamera);
    }

    public synchronized List<RigidTransform2D> getCaptureTimeFieldToGoal() {
        List<RigidTransform2D> rv = new ArrayList<>();
        for (GoalTracker.TrackReport report : _goalTracker.getTracks()){
            rv.add(RigidTransform2D.fromTranslation(report.fieldToGoal));
        }
        return rv;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform2D observation) {
        _fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservation(double timestamp, Twist2D measuredVelocity, Twist2D predictedVelocity){
        addFieldToVehicleObservation(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measuredVelocity));
        _vehicleVelocityMeasured = measuredVelocity;
        _vehicleVelocityPredicted = predictedVelocity;
    }

    public void addVisionUpdate(double timestamp, List<TargetInfo> visionUpdate) {
        List<Translation2D> fieldToGoals = new ArrayList<>();
        RigidTransform2D fieldToCamera = getFieldToCamera(timestamp);
        if (!(visionUpdate == null || visionUpdate.isEmpty())) {
            for (TargetInfo target : visionUpdate) {
                double yDeadband = target.getY();

                // Compensate for camera Yaw
                double xYaw = target.getX() * _cameraYawCorrection.getCos() + yDeadband * _cameraYawCorrection.getSin();
                double yYaw = yDeadband * _cameraYawCorrection.getCos() - target.getX() * _cameraYawCorrection.getSin();
                double zYaw = target.getZ();

                // Compensate for camera Pitch
                double xr = zYaw * _cameraPitchCorrection.getSin() + xYaw * _cameraPitchCorrection.getCos();
                double yr = yYaw;
                double zr = zYaw * _cameraPitchCorrection.getCos() - xYaw * _cameraPitchCorrection.getSin();

                double scaling =_differentialHeight / zr;
                double distance = Math.hypot(xr, yr) * scaling;
                Rotation2D angle = new Rotation2D(xr, yr, true);
                fieldToGoals.add(fieldToCamera.transformBy(RigidTransform2D.fromTranslation(new Translation2D(distance * angle.getCos(), distance * angle.getSin()))).getTranslation());
            }
        }

        synchronized (this) {
            _goalTracker.update(timestamp, fieldToGoals);
        }
    }

    public synchronized Optional<AimingParameters> getCachedAimingParameters() {
        return _cachedAimingParameters == null ? Optional.empty() : Optional.of(_cachedAimingParameters);
    }

    public synchronized Optional<AimingParameters> getAimingParameters() {
        List<GoalTracker.TrackReport> reports = _goalTracker.getTracks();
        if (!reports.isEmpty()) {
            GoalTracker.TrackReport report = reports.get(0);
            Translation2D robotToGoal = getLatestFieldToVehicle().getValue().getTranslation().inverse().translateBy(report.fieldToGoal);
            Rotation2D robotToGoalRotation = Rotation2D.fromRadians(Math.atan2(robotToGoal.getY(), robotToGoal.getX()));

            AimingParameters parameters = new AimingParameters(robotToGoal.norm(), robotToGoalRotation, report.latestTimestamp, report.stability);
            _cachedAimingParameters = parameters;

            return Optional.of(parameters);
        } else {
            return Optional.empty();
        }
    }
}


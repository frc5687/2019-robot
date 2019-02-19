package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.Limelight;
import static org.frc5687.deepspace.robot.Constants.Auto.DriveToTarget.*;

public class AutoDriveToTarget extends OutliersCommand {
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private OI _oi;

    private PIDController _angleController;
    private PIDController _distanceController;

    private double _angleTarget;
    private double _distanceTarget;
    private double _distanceTolerance;

    private double _speed;
    private double _currentTargetDistance;

    private double _anglePIDOut;
    private double _distancePIDOut;

    private long _startTimeMillis;
    private boolean _aborted = false;

    private String _stage = "";

    public AutoDriveToTarget (Robot robot, double speed, double distance, double tolerance, String stage) {
        _driveTrain = robot.getDriveTrain();
        _imu = robot.getIMU();
        _limelight = robot.getLimelight();
        _oi = robot.getOI();

        requires(_driveTrain);
        _speed = speed;
        _distanceTarget = distance;
        _distanceTolerance = tolerance;
        _stage = stage;
    }

    @Override
    protected void initialize() {
        _aborted = false;
        _driveTrain.resetDriveEncoders();
        _startTimeMillis = System.currentTimeMillis();
        error("Running AutoDriveToTarget to " + _distanceTarget + " inches at " + _speed);

        boolean irMode = true;

        _currentTargetDistance = _driveTrain.getFrontDistance();
        if (_currentTargetDistance > Constants.Auto.IR_THRESHOLD) {
            irMode = false;
            if (_limelight.isTargetSighted()) {
                _currentTargetDistance = _limelight.getTargetDistance();
            } else {
                _aborted = true;
            }
        }
        metric("distance/IRMode", irMode);

        double distanceSetPoint = _driveTrain.getDistance() + _currentTargetDistance - _distanceTarget;

        _distanceController = new PIDController(kPDistance, kIDistance, kDDistance, _driveTrain, new DistanceListener(), 0.1);
        _distanceController.setOutputRange(-_speed, _speed);
        _distanceController.setAbsoluteTolerance(_distanceTolerance);
        _distanceController.setContinuous(false);
        _distanceController.setSetpoint(distanceSetPoint);
        _distanceController.enable();

        metric("distance/setpoint", distanceSetPoint);
        metric("distance/currentTargetDistance", _currentTargetDistance);
        metric("distance/distanceTarget", _distanceTarget);

        // 1: Read current target _angleTarget from limelight
        // 2: Read current yaw from navX
        // 3: Set _angleController._angleTarget to sum

        // If we can't see the target, don't use limelight's angle!
        double limeLightAngle = irMode ? 0 : _limelight.getHorizontalAngle();
        double yawAngle = _imu.getYaw();
        _angleTarget = limeLightAngle + yawAngle;

        metric ("angle/startoffset", limeLightAngle);
        metric("angle/startyaw", yawAngle);
        metric("angle/target", _angleTarget);

        _angleController = new PIDController(kPAngle, kIAngle, kDAngle, _imu, new AngleListener(), 0.1);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-TURN_SPEED, TURN_SPEED);
        _angleController.setAbsoluteTolerance(ANGLE_TOLERANCE);
        _angleController.setContinuous();
        _angleController.setSetpoint(_angleTarget);
        _angleController.enable();
    }

    @Override
    protected void execute() {
        boolean irMode = true;

        _currentTargetDistance = _driveTrain.getFrontDistance();
        if (_currentTargetDistance > Constants.Auto.IR_THRESHOLD) {
            irMode = false;
            if (_limelight.isTargetSighted()) {
                _currentTargetDistance = _limelight.getTargetDistance();
            }
        }

        double distanceSetPoint = _driveTrain.getDistance() + _currentTargetDistance - _distanceTarget;
        double oldSetpoint = _distanceController.getSetpoint();

        if (Math.abs(distanceSetPoint - oldSetpoint) > _distanceTolerance) {
            _distanceController.setSetpoint(distanceSetPoint);
            metric("distance/setpoint", distanceSetPoint);
            oldSetpoint = distanceSetPoint;
            _distanceController.enable();
        }

        if (_speed > 0.3 && Math.abs(oldSetpoint - _driveTrain.getDistance()) <= 36) {
            DriverStation.reportError("CAPPING", false);
            _distanceController.setOutputRange(-0.3, 0.3);
            _distanceController.enable();
        }


        if (!irMode && _limelight.isTargetSighted()) {
            double limeLightAngle = _limelight.getHorizontalAngle();
            double yawAngle = _imu.getYaw();
            _angleTarget = limeLightAngle + yawAngle;

            metric("angle/startoffset", limeLightAngle);
            metric("angle/startyaw", yawAngle);
            metric("angle/target", _angleTarget);

            if (Math.abs(_angleTarget - _angleController.getSetpoint()) > Constants.Auto.DriveToTarget.ANGLE_TOLERANCE) {
                _angleController.setSetpoint(_angleTarget);
                metric("angle/setpoint", _angleTarget);
            }
        }

        metric("angle/yaw", _imu.getYaw());

        metric("distance/currentTargetDistance", _currentTargetDistance);
        metric("angle/PIDOut", _anglePIDOut);
        metric("distance/PIDOut", _distancePIDOut);
        metric("distance/target", distanceSetPoint);
        metric("distance/current", _driveTrain.getDistance());
        metric("distance/IRMode", irMode);
        _driveTrain.setPower(_distancePIDOut + _anglePIDOut , _distancePIDOut - _anglePIDOut, true); // positive output is clockwise

    }
    @Override
    protected boolean isFinished() {
        if (_aborted) { return true; }

        if (_distanceController.onTarget()) {
            return true;
        }

        if (Math.abs(_currentTargetDistance - _distanceTarget) <= _distanceTolerance) { return true; }

        return false;
    }

    @Override
    protected void end() {
        _driveTrain.enableBrakeMode();
        _driveTrain.setPower(0,0, true);
        error("AutoDriveToTarget finished: angle=" + _imu.getYaw() + ", distance=" + _currentTargetDistance + ", time=" + (System.currentTimeMillis() - _startTimeMillis));
        _distanceController.disable();
        _angleController.disable();
    }

    private class AngleListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }
    }

    private class DistanceListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _distancePIDOut = output;
            }
        }
    }
}

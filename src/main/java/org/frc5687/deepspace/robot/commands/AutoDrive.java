package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;

public class AutoDrive extends OutliersCommand {
    private double _distance;
    private double _speed;
    private PIDController _distanceController;
    private PIDController _angleController;
    private double _distancePIDOut;
    private double _anglePIDOut;

    private boolean _usePID;
    private boolean _stopOnFinish;
    private double _angle;
    private String _stage;

    private DriveTrain _driveTrain;
    private AHRS _imu;

    private double kPdistance = 0.15; // .05;
    private double kIdistance = 0.000; // .001;
    private double kDdistance = 0.3; //.1;
    private double kTdistance = 0.5;

    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private double kTangle;


    /***
     * Drives for a set distance at a set speed.
     * @param distance Distance to drive
     * @param speed Speed to drive
     * @param usePID Whether to use pid or not
     * @param stopOnFinish Whether to stop the motors when we are done
     * @param angle The angle to drive, in degrees.  Pass 1000 to maintain robot's hading.
     */
    public AutoDrive(DriveTrain driveTrain, AHRS imu, double distance, double speed, boolean usePID, boolean stopOnFinish, double angle, String stage, double timeout) {
        super(timeout);
        requires(driveTrain);
        _speed = speed;
        _distance = distance;
        _usePID = usePID;
        _stopOnFinish = stopOnFinish;
        _angle = angle;
        _stage = stage;
        _driveTrain = driveTrain;
        _imu = imu;
    }

    @Override
    protected void initialize() {
        _driveTrain.resetDriveEncoders();

        _driveTrain.enableBrakeMode();
        if (_usePID) {
            metric("kP", kPdistance);
            metric("kI", kIdistance);
            metric("kD", kDdistance);
            metric("kT", kTdistance);

            _distanceController = new PIDController(kPdistance, kIdistance, kDdistance, _speed, _driveTrain, new DistanceListener(), 0.01);
            _distanceController.setAbsoluteTolerance(kTdistance);
            _distanceController.setOutputRange(-_speed, _speed);
            _distanceController.setSetpoint(_driveTrain.getDistance() + _distance);
            _distanceController.enable();
        }

        _angleController = new PIDController(kPangle, kIangle, kDangle, _imu, new AngleListener(), 0.01);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        double maxSpeed = _speed * Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE;
        metric("angleMaxSpeed", maxSpeed);
        metric("setPoint", _driveTrain.getYaw());
        _angleController.setOutputRange(-maxSpeed, maxSpeed);
        _angleController.setContinuous();

        // If an angle is supplied, use that as our setpoint.  Otherwise get the current heading and stick to it!
        _angleController.setSetpoint(_angle ==1000? _driveTrain.getYaw(): _angle);
        _angleController.enable();

        info("Auto Drive initialized: " + (_stage ==null?"": _stage));
    }

    @Override
    protected void execute() {
        double baseSpeed = _usePID ? _anglePIDOut : _speed;
        _driveTrain.setPower(baseSpeed + _anglePIDOut , baseSpeed - _anglePIDOut, true); // positive output is clockwise
        metric("onTarget", _distanceController == null ? false : _distanceController.onTarget());
        metric("imu", _driveTrain.getYaw());
        metric("distance", _driveTrain.pidGet());
        metric("turnPID", _anglePIDOut);
        metric("distancePID", _distancePIDOut);
    }

    @Override
    protected boolean isFinished() {
        if (_usePID) {
            if (_distanceController.onTarget()) {
               DriverStation.reportError("AutoDrive stoped at " + _driveTrain.getDistance(),false);
            }
        } else {
            info("AutoDrive nopid complete at " + _driveTrain.getDistance() + " inches");
            return _distance == 0 ? true : _distance < 0 ? (_driveTrain.getDistance() < _distance) : (_driveTrain.getDistance() > _distance);
        }
        return false;
    }



    @Override
    protected void end() {
        if (isTimedOut()) {
            warn("AutoDrive Finished (" + _driveTrain.getDistance() + ", " + (_driveTrain.getYaw() - _angleController.getSetpoint()) + ") " + (_stage ==null?"": _stage));
        } else {
            info("AutoDrive Finished (" + _driveTrain.getDistance() + ", " + (_driveTrain.getYaw() - _angleController.getSetpoint()) + ") " + (_stage == null ? "" : _stage));
        }
        DriverStation.reportError("AutoDrive Finished (" + _driveTrain.getDistance() + ", " + (_driveTrain.getYaw() - _angleController.getSetpoint()) + ") " + (_stage ==null?"": _stage), false);
        _driveTrain.enableCoastMode();
        _angleController.disable();
        if (_distanceController !=null) {
            _distanceController.disable();
        }
        if (_stopOnFinish) {
            info("Stopping at ." + _driveTrain.getDistance());
            _driveTrain.enableBrakeMode();
            _driveTrain.setPower(0, 0, true);
        }
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

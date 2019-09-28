package org.frc5687.deepspace.robot;

import org.frc5687.deepspace.robot.subsystems.DriveTrain;

public class RobotPos {

    private DriveTrain  _driveTrain;

    private double _leftDriveSpeedFPS;
    private double _rightDriveSpeedFPS;
    private double _lastTime  = 0.0;
    private double _deltaTime = 20.0;

    private boolean _firstCycle = true;
    private boolean _usingNavX = true;


    private double _xPosition = 0;
    private double _yPosition = 0;
    private double _autoStartAngle = 90;

    private double _gyroAngle;
    private double _lastGyroAngle;
    private double _driveVelocityState = 0;


    public RobotPos(){



    }

    public void update() {

        if (_lastTime == 0.0) {
            _deltaTime = 20.0;
            _lastTime = System.currentTimeMillis();
        } else {
            _deltaTime = System.currentTimeMillis() - _lastTime;
            _lastTime = System.currentTimeMillis();
        }
        if (_firstCycle) {
            _firstCycle = false;
            if (_usingNavX) {
                _lastGyroAngle = _driveTrain.getYaw() + (_autoStartAngle - 90);
            } else {
                _lastGyroAngle = 0.0;
            }
            _driveVelocityState = 0;
        } else {
            _driveVelocityState = this.getDriveSpeedFPS();
        }

        double leftTicksPerCycle = this.getEncoderLeftSpeed();
        double rightTicksPerCycle = this.getEncoderRightSpeed();
        _leftDriveSpeedFPS =(((leftTicksPerCycle / 1024) * (Math.PI * DIAMETER))/ 12.0) * (1000.0 / _deltaTime);
        _rightDriveSpeedFPS =(((rightTicksPerCycle / 1024) * (Math.PI * DIAMETER))/ 12.0) * (1000.0 / _deltaTime);


        if (_usingNavX) {
            _gyroAngle = _driveTrain.getYaw() + (_autoStartAngle - 90);
        } else {
            _gyroAngle = 0.0;
        }

        double driveXSpeed = _driveVelocityState * Math.cos(Math.toRadians(this.gyroPositionState));
        double driveYSpeed = _driveVelocityState * Math.sin(Math.toRadians(this.gyroPositionState));
        _xPosition += driveXSpeed * _deltaTime / 1000.0;
        _yPosition += driveYSpeed * _deltaTime / 1000.0;
    }

    public double getEncoderLeftSpeed() {
        return _driveTrain.getLeftPower();
    }
    public double getEncoderRightSpeed() {
        return _driveTrain.getRightPower();
    }
    public double getDriveSpeedFPS() {
        return (_leftDriveSpeedFPS + _rightDriveSpeedFPS) / 2.0;
    }




}

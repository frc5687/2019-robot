package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.Limelight;


import static org.frc5687.deepspace.robot.Constants.Auto.AutoAlignWhileDriving.*;

public class AutoAlignWhileDriving extends OutliersCommand {
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private OI _oi;

    private PIDController _pidController;

    private double _PIDOut;
    private double _angleTarget;
    private double _speed;
    private double _tolorance;

    public AutoAlignWhileDriving(Robot robot, double speed, double tolorance) {
        _driveTrain = robot.getDriveTrain();
        _imu = robot.getIMU();
        _oi = robot.getOI();

        requires(_driveTrain);
        _tolorance = tolorance;
        _speed = speed;


    }

    protected void initialize() {
        _driveTrain.resetDriveEncoders();

        double limeLightAngle = _limelight.getHorizontalAngle();
        double yawAngle = _imu.getYaw();
        _angleTarget = limeLightAngle + yawAngle;

        metric ("angle/startoffset", limeLightAngle);
        metric("angle/startyaw", yawAngle);
        metric("angle/target", _angleTarget);

        _pidController = new PIDController(kP, kI, kD, _imu, new PIDListener(), 0.1);
        _pidController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _pidController.setOutputRange(-_speed, _speed);
        _pidController.setAbsoluteTolerance(_tolorance);
        _pidController.setContinuous();
        _pidController.setSetpoint(_angleTarget);
        _pidController.enable();

    }

    protected void execute() {

        double stickSpeed = _oi.getDriveSpeed();
        if (_limelight.isTargetSighted()) {
            double limeLightAngle = _limelight.getHorizontalAngle();
            double yawAngle = _imu.getYaw();
            _angleTarget = limeLightAngle + yawAngle;

            metric("angle/startoffset", limeLightAngle);
            metric("angle/startyaw", yawAngle);
            metric("angle/target", _angleTarget);

            if (Math.abs(_angleTarget - _pidController.getSetpoint()) > _tolorance) {
                _pidController.setSetpoint(_angleTarget);
                metric("angle/setpoint", _angleTarget);
            }
        }
        _driveTrain.cheesyDrive(stickSpeed, _PIDOut);
    }

    protected boolean isFinished() {
        if(_pidController.onTarget()) {
            return true;
        }

        return false;}

    protected void end() {}

    private class PIDListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _PIDOut = output;
            }
        }

    }
}

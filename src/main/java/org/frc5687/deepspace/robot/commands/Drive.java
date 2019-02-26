package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.Limelight;

import static org.frc5687.deepspace.robot.Constants.Auto.AutoAlignWhileDriving.*;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;

    private PIDController _pidController;

    private double _PIDOut;
    private double _angleTarget;


    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        requires(_driveTrain);
    }
    protected void initialize() {
        double limeLightAngle = _limelight.getHorizontalAngle();
        double yawAngle = _imu.getYaw();
        _angleTarget = limeLightAngle + yawAngle;

        metric ("angle/startoffset", limeLightAngle);
        metric("angle/startyaw", yawAngle);
        metric("angle/target", _angleTarget);

        _pidController = new PIDController(kP, kI, kD, _imu, new PIDListener(), 0.1);
        _pidController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _pidController.setOutputRange(-TURN_SPEED, TURN_SPEED);
        _pidController.setAbsoluteTolerance(TOLORANCE);
        _pidController.setContinuous();
        _pidController.setSetpoint(_angleTarget);
        _pidController.enable();

    }

    @Override
    protected void execute() {
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
        double yAxisSpeed = _oi.getDriverRightYAxix();
        if (yAxisSpeed > DEADBAND && _limelight.isTargetSighted()) {
            double limeLightAngle = _limelight.getHorizontalAngle();
            double yawAngle = _imu.getYaw();
            _angleTarget = limeLightAngle + yawAngle;

            metric("angle/startoffset", limeLightAngle);
            metric("angle/startyaw", yawAngle);
            metric("angle/target", _angleTarget);

            if (Math.abs(_angleTarget - _pidController.getSetpoint()) > TOLORANCE) {
                _pidController.setSetpoint(_angleTarget);
                metric("angle/setpoint", _angleTarget);
            }
        }
        _driveTrain.cheesyDrive(stickSpeed, _PIDOut, true);

        _driveTrain.cheesyDrive(stickSpeed, wheelRotation, false);
    }



    @Override
    protected boolean isFinished() {
        return false;
    }

    private class PIDListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _PIDOut = output;
            }
        }

    }
}

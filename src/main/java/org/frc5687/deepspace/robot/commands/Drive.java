package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.Elevator;
import org.frc5687.deepspace.robot.utils.BasicPose;
import org.frc5687.deepspace.robot.utils.Limelight;

import static org.frc5687.deepspace.robot.Constants.Auto.Align.*;
import org.frc5687.deepspace.robot.utils.PDP;
import org.frc5687.deepspace.robot.utils.PoseTracker;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private PoseTracker _poseTracker;
    private Elevator _elevator;

    private PIDController _angleController;

    private double _anglePIDOut;
    private double _angle;
    private double _turnSpeed;
    private boolean _autoAlignEnabled = false;
    private boolean _targetSighted;

    public Drive(DriveTrain driveTrain, AHRS imu, OI oi, Limelight limelight, Elevator elevator, PoseTracker poseTracker) {
        _driveTrain = driveTrain;
        _oi = oi;
        _imu = imu;
        _limelight = limelight;
        _elevator = elevator;
        _poseTracker = poseTracker;
        requires(_driveTrain);

        logMetrics("StickSpeed", "StickRotation", "LeftPower", "RightPower", "LeftMasterAmps", "LeftFollowerAmps", "RightMasterAmps", "RightFollowerAmps", "TurnSpeed");
    }



    @Override
    protected void initialize() {
        // create the _angleController here, just like in AutoDriveToTarget
        _autoAlignEnabled = false;
        _targetSighted = false;
        _angleController = new PIDController(kP,kI,kD, _imu, new AngleListener(), 0.1);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-SPEED, SPEED);
        _angleController.setAbsoluteTolerance(TOLERANCE);
        _angleController.setContinuous();
    }

    @Override
    protected void execute() {
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
        _targetSighted = _limelight.isTargetSighted();

        // If the auto-align trigger is pressed, and !_autoAlignEnabled:
        //   Enable the LEDs
        // else if auto_align trigger is not pressed, and _autoAlignEnabled
        //   disable the LEDs, disable the controller
        // else if _autoAlignEnabled
        //   Get target info (copy from AutoAlignToTarget)
        //   If target sighted and ither controller not enabled or new setpoint different enough from old setpoint
        //      set setPoint
        //      enable controller
        if (!_autoAlignEnabled && _oi.isAutoTargetPressed() && _elevator.isLimelightClear()) {
            _limelight.enableLEDs();
            _autoAlignEnabled = true;
        } else if (_autoAlignEnabled &&(!_oi.isAutoTargetPressed() || !_elevator.isLimelightClear())) {
            _limelight.disableLEDs();
            _angleController.disable();
            _autoAlignEnabled = false;
        } else if (_autoAlignEnabled && _targetSighted) {
            double limeLightAngle = _limelight.getHorizontalAngle();
            double yawAngle = _imu.getYaw();
            _angle = limeLightAngle + yawAngle;
            if (!_angleController.isEnabled() || Math.abs(_angle - _angleController.getSetpoint()) > TOLERANCE) {
                _angleController.setSetpoint(_angle);
                _angleController.enable();
                metric("limelightOffset", limeLightAngle);
                metric("target", _angle);
            }

            long timeKey = System.currentTimeMillis() - (long)_limelight.getLatency();
            BasicPose pose = (BasicPose)_poseTracker.get(timeKey);
            double poseAngle = pose == null ? _imu.getYaw() : pose.getAngle();
            double offsetCompensation = _imu.getYaw() - poseAngle;
            double targetAngle = _limelight.getHorizontalAngle() - offsetCompensation;
            _turnSpeed = targetAngle * STEER_K;
            metric("Pose", pose==null?0:pose.getMillis());
        }

//         If autoAlignEnabled and pidControllerEnabled, send pidOut in place of wheelRotation (you may need a scale override flag as discussed earlier)
        if (_autoAlignEnabled && _angleController.isEnabled()) {
            //_driveTrain.cheesyDrive(stickSpeed, _anglePIDOut, false, true);
            _driveTrain.cheesyDrive(stickSpeed, _turnSpeed, false, true);
        } else {
            _driveTrain.cheesyDrive(stickSpeed, wheelRotation, _oi.isCreepPressed(), false);
        }
        metric("StickSpeed", stickSpeed);
        metric("StickRotation", wheelRotation);
        metric("LeftPower", _driveTrain.getLeftPower());
        metric("RightPower", _driveTrain.getRightPower());
        metric("LeftMasterAmps", _driveTrain.getLeftMasterCurrent());
        metric("LeftFollowerAmps",_driveTrain.getLeftFollowerCurrent());
        metric("RightMasterAmps",_driveTrain.getRightMasterCurrent());
        metric("RightFollowerAmps",_driveTrain.getRightFollowerCurrent());
        metric("TurnSpeed", _turnSpeed);
    }



    @Override
    protected boolean isFinished() {
        return false;
    }

    private class AngleListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }

    }
}

package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.Elevator;
import org.frc5687.deepspace.robot.subsystems.HatchIntake;
import org.frc5687.deepspace.robot.utils.BasicPose;
import org.frc5687.deepspace.robot.utils.Helpers;
import org.frc5687.deepspace.robot.utils.Limelight;

import org.frc5687.deepspace.robot.utils.PoseTracker;

import static org.frc5687.deepspace.robot.Constants.Auto.Align.STEER_K;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private PoseTracker _poseTracker;
    private Elevator _elevator;
    private HatchIntake _hatchIntake;
    private CargoIntake _cargoIntake;

    private PIDController _angleController;

    private double _anglePIDOut;
    private double _angle;
    private double _turnSpeed;
    private boolean _targetSighted;
    private long _lockEnd;
    private DriveState _driveState = DriveState.normal;

    private long _seekMax;


    public Drive(DriveTrain driveTrain, AHRS imu, OI oi, Limelight limelight, Elevator elevator, CargoIntake cargoIntake,HatchIntake hatchIntake, PoseTracker poseTracker) {
        _driveTrain = driveTrain;
        _oi = oi;
        _imu = imu;
        _limelight = limelight;
        _elevator = elevator;
        _poseTracker = poseTracker;
        _cargoIntake = cargoIntake;
        _hatchIntake = hatchIntake;
        requires(_driveTrain);

        // logMetrics("StickSpeed", "StickRotation", "LeftPower", "RightPower", "LeftMasterAmps", "LeftFollowerAmps", "RightMasterAmps", "RightFollowerAmps", "TurnSpeed");
    }



    @Override
    protected void initialize() {
        // create the _angleController here, just like in AutoDriveToTarget
        _driveState = DriveState.normal;
        _targetSighted = false;
        _angleController = new PIDController(Constants.Auto.Drive.AnglePID.kP,Constants.Auto.Drive.AnglePID.kI,Constants.Auto.Drive.AnglePID.kD, _imu, new AngleListener(), 0.1);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE, Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE);
        _angleController.setAbsoluteTolerance(Constants.Auto.Drive.AnglePID.TOLERANCE);
        _angleController.setContinuous();
    }

    @Override
    protected void execute() {
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
        _targetSighted = _limelight.isTargetSighted();
        if (!_oi.isAutoTargetPressed() || !_elevator.isLimelightClear()) {
            if (_driveState!=DriveState.normal) {
                // Stop tracking
                _limelight.disableLEDs();
                _driveState = DriveState.normal;
            }
            if (wheelRotation==0 && stickSpeed != 0) {
                // Driving straight...lock in the yaw
                if (!_angleController.isEnabled()) {
                    _anglePIDOut = 0;
                    double yaw = _imu.getYaw();
                    metric("PID/SetPoint", yaw);
                    _angleController.setSetpoint(yaw);
                    _angleController.setPID(Constants.Auto.Drive.AnglePID.kP, Constants.Auto.Drive.AnglePID.kI, Constants.Auto.Drive.AnglePID.kD);
                    _angleController.enable();
                    metric("PID/Enabled", true);
                }
            } else if (_angleController.isEnabled()) {
                _angleController.disable();
                _anglePIDOut = 0;
                metric("PID/Enabled", false);
            }
        } else {
            switch (_driveState) {
                case normal:
                    // Start seeking
                    _limelight.setPipeline(_cargoIntake.isIntaking() ? 8 : 0);
                    _limelight.enableLEDs();
                    _driveState = DriveState.seeking;
                    _seekMax = System.currentTimeMillis() + Constants.DriveTrain.SEEK_TIME;
                    break;
                case seeking:
                    if (_targetSighted) {
                        _lockEnd = System.currentTimeMillis() + Constants.DriveTrain.LOCK_TIME;
                        _driveState = DriveState.locking;
                    }
                    break;
                case locking:
                    if (System.currentTimeMillis() > _lockEnd) {
                        // Note that we could also wait until the target is centered to lock...which might make more sense.
                        // Just add  && _limelight.isTargetCentered() to the condition above
                        _limelight.setPipeline(_cargoIntake.isIntaking() ? 9 : 1);
                        _driveState = DriveState.tracking;
                    }
                    _turnSpeed = getTurnSpeed();
                    break;
                case tracking:
                    _turnSpeed = getTurnSpeed();
                    break;
            }
        }
        // If the auto-align trigger is pressed, and !_autoAlignEnabled:
        //   Enable the LEDs
        // else if auto_align trigger is not pressed, and _autoAlignEnabled
        //   disable the LEDs, disable the controller
        // else if _autoAlignEnabled
        //   Get target info (copy from AutoAlignToTarget)
        //   If target sighted and ither controller not enabled or new setpoint different enough from old setpoint
        //      set setPoint
        //      enable controller

//         If autoAlignEnabled and pidControllerEnabled, send pidOut in place of wheelRotation (you may need a scale override flag as discussed earlier)
        stickSpeed = limitSpeed(stickSpeed);
        if(!_oi.isOverridePressed() && _hatchIntake.isShockTriggered()) {
            _driveTrain.cheesyDrive(Math.min(stickSpeed, 0), 0, false, false);
        } else if (_driveState == DriveState.normal) {
            if (wheelRotation==0 && _angleController.isEnabled()) {
                metric("PID/AngleOut", _anglePIDOut);
                metric("PID/Yaw", _imu.getYaw());
                _driveTrain.cheesyDrive(stickSpeed, stickSpeed==0 ?  0 :_anglePIDOut, false, true);
            } else {
                _driveTrain.cheesyDrive(stickSpeed, wheelRotation, _oi.isCreepPressed(), false);
            }
        } else {
            _driveTrain.cheesyDrive(stickSpeed, _turnSpeed, false, true);
        }
        metric("StickSpeed", stickSpeed);
        metric("StickRotation", wheelRotation);
        metric("LeftPower", _driveTrain.getLeftPower());
        metric("RightPower", _driveTrain.getRightPower());
        // metric("LeftMasterAmps", _driveTrain.getLeftMasterCurrent());
        // metric("LeftFollowerAmps",_driveTrain.getLeftFollowerCurrent());
        // metric("RightMasterAmps",_driveTrain.getRightMasterCurrent());
       //  metric("RightFollowerAmps",_driveTrain.getRightFollowerCurrent());
        metric("TurnSpeed", _turnSpeed);
    }

    protected double getTurnSpeed() {
        double limeLightAngle = _limelight.getHorizontalAngle();
        double yaw = _imu.getYaw();

        // Find the pose of the robot _when the picture was taken_
        long timeKey = System.currentTimeMillis() - (long)_limelight.getLatency();
        BasicPose pose = (BasicPose)_poseTracker.get(timeKey);

        // Get the angle from the pose if one was found--otherwise use yaw
        double poseAngle = pose == null ? yaw : pose.getAngle();

        // Now adjust the limelight angle based on the change in yaw from when the picture was taken to now
        double offsetCompensation = yaw - poseAngle;
        double targetAngle = limeLightAngle - offsetCompensation;

        metric("Pose", pose==null?0:pose.getMillis());
        metric("Yaw", yaw);
        metric("PoseAngle", poseAngle);
        metric("LimelightAngle", limeLightAngle);
        metric("TargetAngle", targetAngle);

        return targetAngle * STEER_K;
    }

    private double limitSpeed(double speed) {
        double limit = 1;
        if (_driveState!=DriveState.normal) {
            if(_limelight.isTargetSighted()) {
                double distance = _limelight.getTargetDistance();
                metric("distance", distance);
                 if (distance  < 100) limit = 0.60;
                 if (distance  < 20) limit = 0.30;
            } else if (System.currentTimeMillis() > _seekMax){
                // We've been seeking for more than the max allowed...slow the robot down!
                limit = 0.75;
                _oi.pulseDriver(1);
            }
        }
        if (_elevator.isAboveMiddle()) {
            limit = Math.min(0.5, limit);
        }
        double limited = Helpers.limit(speed, -limit, limit);
        metric("limit", limit);
        metric("scaled", limited);
        return limited;
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

    public enum DriveState {
        normal(0),
        seeking(1),
        locking(2),
        tracking(3),
        lost(4);

        private int _value;

        DriveState(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}

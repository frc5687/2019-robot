package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.Limelight;
import org.frc5687.deepspace.robot.utils.RioLogger;

import static org.frc5687.deepspace.robot.Constants.Auto.Align.*;


public class AutoAlignToTarget extends OutliersCommand implements PIDOutput {

    private PIDController _controller;
    private double _angle;
    private double _speed;
    private long _timeout = 2000;

    private double _pidOut;

    private long _onTargetSince;
    private long _startTimeMillis;
    private long _endTimeMillis;
    private boolean _targetSighted = false;
    private boolean _aborted = false;

    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;

    private String _stage = "";


    private OI _oi;


    private double _tolerance;


    public AutoAlignToTarget(DriveTrain driveTrain, OI oi, AHRS imu, Limelight limelight, double speed, long timeout, double tolerance, String stage) {
        requires(driveTrain);
        _limelight = limelight;
        _oi = oi;
        _speed = speed;
        _driveTrain = driveTrain;
        _imu = imu;
        _timeout = timeout;
        _tolerance = tolerance;
        _stage = stage;
    }

    @Override
    protected void initialize() {
        _driveTrain.enableBrakeMode();
        _limelight.enableLEDs();
        _targetSighted = false;


        _controller = new PIDController(kP, kI, kD, _imu, this, 0.01);

//        double limeLightAngle = _limelight.getHorizontalAngle();
//        double yawAngle = _imu.getYaw();
//        _angle = limeLightAngle + yawAngle;
//
//        metric("LimeLightAngle", limeLightAngle);
//        metric("YawAngle", yawAngle);
//        metric("angle", _angle);
//
//        _controller = new PIDController(kP, kI, kD, _imu, this, 0.01);
//        _controller.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
//        _controller.setOutputRange(-_speed, _speed);
//        _controller.setAbsoluteTolerance(_tolerance);
//        _controller.setContinuous();
//        _controller.setSetpoint(_angle);
//        _controller.enable();
//        metric("setpoint", _angle);
//        error("AutoAlign " + _stage + " initialized to " + _angle + " at " + _speed);
//        error("kP="+kP+" , kI="+kI+", kD="+kD + ",T="+ _tolerance);
        _startTimeMillis = System.currentTimeMillis();
        _endTimeMillis = _startTimeMillis + _timeout;


    }

    @Override
    protected void execute() {
        boolean wasSighted = _targetSighted;
        if (!_targetSighted) {
            error("Target not sighted!");
            _targetSighted = _limelight.isTargetSighted();
        }
        metric("targetSighted", _targetSighted);
        metric("limelight sighted", _limelight.isTargetSighted());

        if (_targetSighted) {
            error("Target sighted!");
            double limeLightAngle = _limelight.getHorizontalAngle();
            double yawAngle = _imu.getYaw();
            _angle = limeLightAngle + yawAngle;
            error(" At " + limeLightAngle + " when our angle was " + yawAngle);

            metric("LimeLightAngle", limeLightAngle);
            metric("YawAngle", yawAngle);
            metric("angle", _angle);

            if (!wasSighted) {
                _controller.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
                _controller.setOutputRange(-_speed, _speed);
                _controller.setAbsoluteTolerance(_tolerance);
                _controller.setContinuous();
                _controller.setSetpoint(_angle);
                _controller.enable();

                metric("setpoint", _angle);
                error("AutoAlign " + _stage + " initialized to " + _angle + " at " + _speed);
            }
        }
        double turn = _pidOut;
        if(_targetSighted) {
            if (turn < 0 && turn > -MINIMUM_SPEED) {
                turn = -MINIMUM_SPEED;
            } else if (turn > 0 && turn < MINIMUM_SPEED) {
                turn = MINIMUM_SPEED;
            }

            error("Sending left=" + _pidOut + ", right=" + (-_pidOut));
            metric("pidOut", _pidOut);
            metric("turn", turn);
            metric("onTarget", _controller.onTarget());
            _driveTrain.setPower(turn, -turn, true);
        }
//        double limeLightAngle = _limelight.getHorizontalAngle();
//        double yawAngle = _imu.getYaw();
//        _angle = limeLightAngle + yawAngle;
//
//        metric("startoffset", limeLightAngle);
//        metric("startyaw", yawAngle);
//        metric("target", _angle);
//
//        if (Math.abs(_angle - _controller.getSetpoint()) > _tolerance) {
//            _controller.setSetpoint(_angle);
//            _controller.enable();
//            metric("setpoint", _angle);
//        }
//        metric("onTarget", _controller.onTarget());
//        metric("yaw", _imu.getYaw());
//        metric("pidOut", _pidOut);



    }
    @Override
    protected boolean isFinished() {
        if(System.currentTimeMillis() >= _endTimeMillis){
            error("AutoAlignToTarget timed out after " + _timeout + "ms at " + _imu.getYaw());
            return true;

        }
        if (_targetSighted) {
            if (_controller.onTarget()) {
                error("AutoAlignToTarget on target at " + _imu.getYaw());
                return true;
            }
        }
        return false;
    }
    @Override
    protected void end() {
        _driveTrain.setPower(0, 0, true);
        _driveTrain.disableBrakeMode();
        error("AutoAlign finished: angle = " + _imu.getYaw() + ", time = " + (System.currentTimeMillis() - _startTimeMillis));
        _controller.disable();
        error("AutoAlign.end() controller disabled");
        _limelight.disableLEDs();
    }

    @Override
    public void pidWrite(double output) {
        _pidOut = output;
    }
}

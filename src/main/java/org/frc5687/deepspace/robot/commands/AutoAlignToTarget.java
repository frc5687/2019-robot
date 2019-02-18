package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;


public class AutoAlignToTarget extends OutliersCommand implements PIDOutput {

    private PIDController _controller;
    private double _angle;
    private double _speed;
    private long _timeout = 2000;

    private double _pidOut;

    private long _onTargetSince;
    private long _startTimeMillis;
    private long _endTimeMillis;
    private boolean _aborted = false;

    private DriveTrain _driveTrain;
    private AHRS _imu;

    private String _stage = "";

    NetworkTable _table;

    private OI _oi;


    private double _tolerance;

    public AutoAlignToTarget(Robot robot, double speed, long timeout, double tolerance) {
        this(robot.getDriveTrain(), robot.getIMU(), speed, timeout, tolerance, "");
        _oi = robot.getOI();
    }


    public AutoAlignToTarget(DriveTrain driveTrain, AHRS imu, double speed, long timeout, double tolerance, String stage) {
        requires(driveTrain);
        _angle = _angle;
        _speed = speed;
        _driveTrain = driveTrain;
        _imu = imu;
        _timeout = timeout;
        _tolerance = tolerance;
        _stage = stage;
    }

    @Override
    protected void initialize() {
        double kP = Constants.Auto.AlignToTarget.kP; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kI = Constants.Auto.AlignToTarget.kI; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kD = Constants.Auto.AlignToTarget.kD; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        // 1: Read current target angle from limelight
        // 2: Read current yaw from navX
        // 3: Set controller.angle to sum
        _table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = _table.getEntry("tx");

        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = _imu.getAngle();
        _angle = limeLightAngle + yawAngle;

        metric("LimeLightAngle", limeLightAngle);
        metric("YawAngle", yawAngle);
        metric("angle", _angle);

        _controller = new PIDController(kP, kI, kD, _imu, this, 0.01);
        _controller.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _controller.setOutputRange(-_speed, _speed);
        _controller.setAbsoluteTolerance(_tolerance);
        _controller.setContinuous();
        _controller.setSetpoint(_angle);
        _controller.enable();
        metric("setpoint", _angle);
        error("AutoAlign " + _stage + " initialized to " + _angle + " at " + _speed);
        error("kP="+kP+" , kI="+kI+", kD="+kD + ",T="+ Constants.Auto.AlignToTarget.TOLERANCE);
        _startTimeMillis = System.currentTimeMillis();
        _endTimeMillis = _startTimeMillis + _timeout;


    }

    @Override
    protected void execute() {
        NetworkTableEntry tx = _table.getEntry("tx");
        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = _imu.getAngle();
        _angle = limeLightAngle + yawAngle;

        metric("startoffset", limeLightAngle);
        metric("startyaw", yawAngle);
        metric("target", _angle);

        if (Math.abs(_angle - _controller.getSetpoint()) > _tolerance) {
            _controller.setSetpoint(_angle);
            metric("setpoint", _angle);
        }
        metric("onTarget", _controller.onTarget());
        metric("yaw", _imu.getYaw());

        actOnPidOut();


    }

    private void actOnPidOut() {
        if (_pidOut > 0 && _pidOut < Constants.Auto.AlignToTarget.MINIMUM_SPEED) {
            _pidOut = Constants.Auto.AlignToTarget.MINIMUM_SPEED;
        }
        if (_pidOut < 0 && _pidOut > -Constants.Auto.AlignToTarget.MINIMUM_SPEED) {
            _pidOut = -Constants.Auto.AlignToTarget.MINIMUM_SPEED;
        }
        metric("pidOut", _pidOut);
    }
    @Override
    protected boolean isFinished() {
        if (_aborted) { return true; }
        if (!_controller.onTarget()) {
            _onTargetSince = 0;
        }

        if((_oi!=null && !_oi.isAutoTargetPressed()) && System.currentTimeMillis() >= _endTimeMillis){
            error("AutoAlignToTarget timed out after " + _timeout + "ms at " + _imu.getYaw());
            return true;
        }

        if (_controller.onTarget()) {
            if (_onTargetSince == 0) {
                error("AutoAlignToTarget reached target " + _imu.getYaw());
                _onTargetSince = System.currentTimeMillis();
            }

            if ((_oi!=null && !_oi.isAutoTargetPressed()) && System.currentTimeMillis() > _onTargetSince + Constants.Auto.AlignToTarget.STEADY_TIME) {
                error("AutoAlignToTarget complete after " + Constants.Auto.AlignToTarget.STEADY_TIME + " at " + _imu.getYaw());
                return  true;
            }
        }

        return false;
    }
    @Override
    protected void end() {
        _driveTrain.setPower(0,0, true);
        error("AutoAlign finished: angle = " + _imu.getYaw() + ", time = " + (System.currentTimeMillis() - _startTimeMillis));
        _controller.disable();
        error("AutoAlign.end() controller disabled");

    }
    @Override
    public void pidWrite(double output) {
        _pidOut = output;
//        actOnPidOut();

    }

}

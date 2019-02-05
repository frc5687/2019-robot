package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.Limelight;

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

    private double speed;

    private double _anglePIDOut;
    private double _distancePIDOut;

    private long _startTimeMillis;
    private boolean _aborted = false;

    private String _message = "";

    public AutoDriveToTarget (Robot robot, double speed, double distance, double tolerance, String message) {
        _driveTrain = robot.getDriveTrain();
        _imu = robot.getIMU();
        _limelight = robot.getLimelight();
        _oi = robot.getOI();

        requires(_driveTrain);
        this.speed = speed;
        _distanceTarget = distance;
        _distanceTolerance = tolerance;
        _message = message;
    }

    @Override
    protected void initialize() {
        _startTimeMillis = System.currentTimeMillis();
        error("Running AutoDriveToTarget to " + _distanceTarget + " inches at " + speed);
        double kPAngle = Constants.Auto.DriveToTarget.kPAngle; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kIAngle = Constants.Auto.DriveToTarget.kIAngle; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kDAngle = Constants.Auto.DriveToTarget.kDAngle; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        double kPDistance = Constants.Auto.DriveToTarget.kPDistance; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kIDistance = Constants.Auto.DriveToTarget.kIDistance; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kDDistance = Constants.Auto.DriveToTarget.kDDistance; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        // 1: Read current target _angleTarget from limelight
        // 2: Read current yaw from navX
        // 3: Set _angleController._angleTarget to sum

        double limeLightAngle = _limelight.getHorizontalAngle();
        double yawAngle = _imu.getAngle();
        _angleTarget = limeLightAngle + yawAngle;

        metric ("angle/startoffset", limeLightAngle);
        metric("angle/startyaw", yawAngle);
        metric("angle/target", _angleTarget);

        _angleController = new PIDController(kPAngle, kIAngle, kDAngle, _imu, new AngleListener(), 0.01);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-Constants.Auto.DriveToTarget.TURN_SPEED, Constants.Auto.DriveToTarget.TURN_SPEED);
        _angleController.setAbsoluteTolerance(Constants.Auto.DriveToTarget.ANGLE_TOLERANCE);
        _angleController.setContinuous();
        _angleController.setSetpoint(_angleTarget);
        _angleController.enable();

        double limeLightYAngle = _limelight.getVerticalAngle();
        double fixedAngle = 0.0;
        double angleY = fixedAngle + limeLightYAngle;
        double tanY = Math.tan(angleY * (Math.PI / 180));
        double currentTargetDistance = (Constants.Auto.DriveToTarget.TARGET_HEIGHT + Constants.Auto.DriveToTarget.LIGHT_HEIGHT)/tanY;

        double distanceSetPoint = _driveTrain.getDistance() + currentTargetDistance - _distanceTarget;

        _distanceController = new PIDController(kPDistance, kIDistance, kDDistance, (PIDSource) _driveTrain, new DistanceListener(), 0.1);
        _distanceController.setOutputRange(-speed, speed);
        _distanceController.setAbsoluteTolerance(_distanceTolerance);
        _distanceController.setContinuous(false);
        _distanceController.setSetpoint(distanceSetPoint);
        _distanceController.enable();

        metric("distance/setpoint", distanceSetPoint);


    }

    @Override
    protected void execute() {
        double limeLightAngle = _limelight.getHorizontalAngle();
        double yawAngle = _imu.getAngle();
        _angleTarget = limeLightAngle + yawAngle;

        metric("angle/startoffset", limeLightAngle);
        metric("angle/startyaw", yawAngle);
        metric("angle/target", _angleTarget);

        if (Math.abs(_angleTarget - _angleController.getSetpoint()) > Constants.Auto.DriveToTarget.ANGLE_TOLERANCE) {
            _angleController.setSetpoint(_angleTarget);
            metric("angle/setpoint", _angleTarget);
        }
        double limeLightYAngle = _limelight.getVerticalAngle();
        double fixedAngle = 0.0;
        double angleY = fixedAngle + limeLightYAngle;
        double tanY = Math.tan(angleY * (Math.PI / 180));
        double currentTargetDistance = (Constants.Auto.DriveToTarget.TARGET_HEIGHT + Constants.Auto.DriveToTarget.LIGHT_HEIGHT)/tanY;

        double distanceSetPoint = _driveTrain.getDistance() + currentTargetDistance - _distanceTarget;
        double oldSetpoint = _distanceController.getSetpoint();

        if (Math.abs(distanceSetPoint - oldSetpoint) > _distanceTolerance) {
            _distanceController.setSetpoint(distanceSetPoint);
            metric("distance/setpoint", distanceSetPoint);
        }

        _distanceController.setSetpoint(distanceSetPoint);
        _distanceController.enable();


        //SmartDashboard.putBoolean("AutoDriveToTarget/angle/onTarget", _angleController.onTarget());
        metric("angle/yaw", _imu.getYaw());

        metric("angle/PIDOut", _anglePIDOut);
        metric("distance/PIDOut", _distancePIDOut);
        metric("distance/target", distanceSetPoint);
        metric("/distance/current", _driveTrain.getDistance());

        //_distancePIDOut = 0;
        _driveTrain.setPower(_distancePIDOut + _anglePIDOut , _distancePIDOut - _anglePIDOut, true); // positive output is clockwise

    }
    @Override
    protected boolean isFinished() {
        if (_aborted) { return true; }
        if (_oi.endIfPressed()) { return true; }

        if (_distanceController.onTarget()) {
            return true;
        }

        if (_driveTrain.getDistance() >= _distanceController.getSetpoint()) { return true; }

        return false;
    }

    @Override
    protected void end() {
        double limeLightYAngle = _limelight.getVerticalAngle();
        double fixedAngle = 0.0;
        double angleY = fixedAngle + limeLightYAngle;
        double tanY = Math.tan(angleY * (Math.PI / 180));
        double currentTargetDistance = (Constants.Auto.DriveToTarget.TARGET_HEIGHT + Constants.Auto.DriveToTarget.LIGHT_HEIGHT)/tanY;

        _driveTrain.enableBrakeMode();
        _driveTrain.setPower(0,0, true);
        error("AutoDriveToTarget finished: angle=" + _imu.getYaw() + ", distance=" + currentTargetDistance + ", time=" + (System.currentTimeMillis() - _startTimeMillis));
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

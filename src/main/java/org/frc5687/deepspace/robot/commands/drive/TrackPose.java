package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.RobotPose;
import org.frc5687.deepspace.robot.utils.AutoChooser;
import org.frc5687.deepspace.robot.utils.Limelight;
import org.frc5687.deepspace.robot.utils.TargetInfo;

import static org.frc5687.deepspace.robot.Constants.Positions.*;

public class TrackPose extends OutliersCommand {
    private RobotPose _robotPose;
    private DriveTrain _driveTrain;
    private TargetInfo _targetInfo;
    private Limelight  _limelight;
    private OI _oi;
    private AutoChooser _autoChooser;

    private double _x;
    private double _y;

    private double _heading;

    private double _deltaTime;
    private double _prevTime;

    public TrackPose(RobotPose robotPose, DriveTrain driveTrain, TargetInfo targetInfo, Limelight limelight, OI oi) {
        _robotPose = robotPose;
        _driveTrain = driveTrain;
        _targetInfo = targetInfo;
        _limelight = limelight;
        _oi = oi;
        requires(_robotPose);

        logMetrics("Heading", "X", "Y","Target/X", "Target/Y");
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/TrackPose", true);
        super.initialize();
        _deltaTime = 20.0;
        _prevTime = 0.0;
        _robotPose.reset();
    }

    @Override
    protected void execute() {
        if ( _prevTime == 0.0) {
            _deltaTime = 20.0;
            _prevTime = System.currentTimeMillis();
        } else {
            _deltaTime = System.currentTimeMillis() - _prevTime;
            _prevTime = System.currentTimeMillis();
        }

        _heading = _driveTrain.getYaw();
        _robotPose.setHeading(_heading);


        //use Riemann Sum to update x and y.
        double xSpeed = _driveTrain.getVelocity() * Math.cos(Math.toRadians(_heading));
        double ySpeed = _driveTrain.getVelocity() * Math.sin(Math.toRadians(_heading));
        _x += xSpeed * _deltaTime;
        _y += ySpeed * _deltaTime;
        _robotPose.setXPosition(_x);
        _robotPose.setYPosition(_y);
        _robotPose.getTargetInfo();
        metric("Heading", _robotPose.getHeading());
        metric("X", _robotPose.getXPosition());
        metric("Y", _robotPose.getYPosition());
        metric("Target/X", _targetInfo.getX());
        metric("Target/Y",  _targetInfo.getY());


        // left side of field.
        if (_robotPose.getXPosition() < 0) {
            // using known vision target locations to periodically account for slipping in wheels.
            if (_robotPose.inRange(LEFT_ROCKET_FRONT_ANGLE) && _limelight.isTargetSighted() && _oi.isAutoTargetPressed()) {
                _robotPose.setXPosition(LEFT_ROCKET_FRONT_HATCH_X - _targetInfo.getX());
                _robotPose.setYPosition(LEFT_ROCKET_FRONT_HATCH_Y - _targetInfo.getY());
            }
        }
        // right side of field.
        if (_robotPose.getXPosition() > 0) {
            // using known vision target locations to periodically account for slipping in wheels.
            if (_robotPose.inRange(RIGHT_ROCKET_FRONT_ANGLE) && _limelight.isTargetSighted() && _oi.isAutoTargetPressed()) {
                _robotPose.setXPosition(RIGHT_ROCKET_FRONT_HATCH_X - _targetInfo.getX());
                _robotPose.setYPosition(RIGHT_ROCKET_FRONT_HATCH_Y - _targetInfo.getY());
            }
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }


}

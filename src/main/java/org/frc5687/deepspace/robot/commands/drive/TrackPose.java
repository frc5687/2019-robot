package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.RobotPose;

public class TrackPose extends OutliersCommand {
    private RobotPose _robotPose;
    private DriveTrain _driveTrain;

    private double _x;
    private double _y;

    private double _heading;

    private double _deltaTime;
    private double _prevTime;

    private double _leftVelocity;
    private double _rightVelocity;

    public TrackPose(RobotPose robotPose, DriveTrain driveTrain) {
        _robotPose = robotPose;
        _driveTrain = driveTrain;
        requires(_robotPose);

        logMetrics("Heading", "X", "Y");
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/TrackPose", true);
        super.initialize();
        _deltaTime = 20.0;
        _prevTime = 0.0;
        _leftVelocity = 0.0;
        _rightVelocity = 0.0;
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

//        _robotPose.navxUpdate();


        _heading = _driveTrain.getYaw();
        _robotPose.setHeading(_heading);


        //use Riemann Sum to update x and y.
        double xSpeed = _driveTrain.getVelocity() * Math.cos(Math.toRadians(_heading));
        double ySpeed = _driveTrain.getVelocity() * Math.sin(Math.toRadians(_heading));
        _x += xSpeed * _deltaTime / 1000;
        _y += ySpeed * _deltaTime / 1000;
        _robotPose.setX(_x);
        _robotPose.setY(_y);
        metric("Heading", _robotPose.getHeading());
        metric("X", _robotPose.getX());
        metric("Y", _robotPose.getY());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

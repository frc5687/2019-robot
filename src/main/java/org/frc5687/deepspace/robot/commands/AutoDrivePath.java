package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;

public class AutoDrivePath extends OutliersCommand {
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;

    private Notifier _notifier;
    private DriveTrain _driveTrain;
    private AHRS _imu;

    private PIDController _angleController;
//    private PIDListener _anglePID;
    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private int _index = 0;
    private String _path;

    private boolean _backwards = false;

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, String path) {
        requires(driveTrain);
        _driveTrain = driveTrain;
        _imu = imu;
        _path = path;
        info("Loading trajectories for " + path);
        _leftTrajectory = PathfinderFRC.getTrajectory(_path + ".right");
        _rightTrajectory = PathfinderFRC.getTrajectory(_path + ".left");

        info("Left has " + _leftTrajectory.length() + " segments.");
        info("Right has " + _rightTrajectory.length() + " segments.");

        logMetrics("Segment", "LeftDistance", "RightDistance", "LeftSpeed","RightSpeed","Heading","DesiredHeading","HeadingDifference", "Turn","LeftOutput","RightOutput");

    }

    @Override
    protected void initialize() {
        SmartDashboard.putBoolean("MetricTracker/AutoDrivePath", true);
        super.initialize();
        _driveTrain.resetDriveEncoders();
        info("Allocating followers");
        if (!_backwards) {
            _leftFollower = new DistanceFollower(_leftTrajectory);
            _rightFollower = new DistanceFollower(_rightTrajectory);
        } else {
            _leftFollower = new DistanceFollower(_rightTrajectory);
            _rightFollower = new DistanceFollower(_leftTrajectory);
        }
        info("COnfiguring follower PID");
        _leftFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);
        _rightFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);

        _index = 0;
    }

    @Override
    protected void execute() {
        _index++;
        metric("Segment", _index);
        double leftDistance = _driveTrain.getLeftDistance() * (_backwards ? -1 : 1);
        double rightDistance = _driveTrain.getRightDistance() * (_backwards ? -1 : 1);
        double leftSpeed = _leftFollower.calculate(leftDistance) * (_backwards ? -1 : 1);
        double rightSpeed = _rightFollower.calculate(rightDistance) * (_backwards ? -1 : 1);
        double heading = _imu.getYaw();

        if (_backwards) { heading = Pathfinder.boundHalfDegrees(heading + 180); }

        double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(_leftFollower.getHeading()));
        double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
        double turn =  Constants.AutoDrivePath.K_TURN * (1.0/80.0) * headingDifference;

        metric("LeftDistance",leftSpeed);
        metric("RightDistance", rightSpeed);
        metric("LeftSpeed",leftSpeed);
        metric("RightSpeed", rightSpeed);
        metric("Heading", heading);
        metric("DesiredHeading", desiredHeading);
        metric("HeadingDifference", headingDifference);
        metric("Turn", turn);

        metric("LeftOutput",leftSpeed + turn);
        metric("RightOutput", rightSpeed - turn);

        _driveTrain.setPower(leftSpeed + turn, rightSpeed - turn, true);
    }

    @Override
    protected boolean isFinished() {
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            info("AutoDrivePath finished");
            return true;
        }
        return false;
    }

    @Override
    protected void end() {
        super.end();
        _leftFollower.reset();
        _rightFollower.reset();
        info("Ending AutoDrivePath");
    }

    //    private class PIDListener implements PIDOutput {
//
//        private double value;
//
//        public double get() {
//            return value;
//        }
//
//        @Override
//        public void pidWrite(double output) {
//            value = output;
//        }
//
//    }

}

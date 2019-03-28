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

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, String path) {

        _driveTrain = driveTrain;
        _imu = imu;
        _path = path;
        info("Loading trajectories for " + path);
        _leftTrajectory = PathfinderFRC.getTrajectory(_path + ".left");
        _leftTrajectory = PathfinderFRC.getTrajectory(_path + ".right");

        info("Left has " + _leftTrajectory.length() + " segments.");
        info("Right has " + _leftTrajectory.length() + " segments.");
/*        for (int i = 0; i < _trajectory.length(); i++) {
            Trajectory.Segment s= _trajectory.get(i);
            DriverStation.reportError("Seg " + i + " x=" + s.x + ", pos=" + s.position + ", vel=" + s.velocity + ", acc="+s.acceleration,false);
        }
*/
    }

    @Override
    protected void initialize() {
        super.initialize();
        _driveTrain.resetDriveEncoders();
        _leftFollower = new DistanceFollower(_leftTrajectory);
        _rightFollower = new DistanceFollower(_rightTrajectory);
        _leftFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);
        _rightFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);

        _notifier = new Notifier(this::followPath);
        _notifier.startPeriodic(_leftTrajectory.get(0).dt);

//        _anglePID = new PIDListener();
//        _angleController = new PIDController(kPangle, kIangle, kDangle, _imu, _anglePID, 0.05);
//        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
//        double maxSpeed = _speed * Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE;
//        SmartDashboard.putNumber("AutoDrive/angleMaxSpeed", maxSpeed);
//        SmartDashboard.putNumber("AutoDrive/setPoint", _driveTrain.getYaw());
//        _angleController.setOutputRange(-maxSpeed, maxSpeed);
//        _angleController.setContinuous();
//
//        // If an angle is supplied, use that as our setpoint.  Otherwise get the current heading and stick to it!
//        _angleController.setSetpoint(_driveTrain.getYaw());
//        _angleController.enable();
        _index = 0;
    }

    @Override
    protected void execute() {
//        double distance = _driveTrain.getDistance();
//        _index++;
//        info("Segment " + _index + " target: " + _follower.getSegment().x + " actual " + distance + " vel=" + _follower.getSegment().velocity);
//        double speed = _follower.calculate(distance);
//        double angleFactor = _anglePID.get();
//
//        info("Calculated speed: " + speed + " anglFactor " + angleFactor);
//
//        _driveTrain.setPower(speed , speed, true);
    }

    private void followPath() {
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            _notifier.stop();
        } else {
            double leftSpeed = _leftFollower.calculate(_driveTrain.getLeftDistance());
            double rightSpeed = _rightFollower.calculate(_driveTrain.getRightDistance());
            double heading = _imu.getYaw();
            double desiredHeading = Pathfinder.r2d(_leftFollower.getHeading());
            double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
            double turn =  Constants.AutoDrivePath.K_TURN * (-1.0/80.0) * headingDifference;
            _driveTrain.setPower(leftSpeed, rightSpeed, true);
        }
    }

    @Override
    protected boolean isFinished() {
        return _leftFollower.isFinished() || _rightFollower.isFinished();
    }

    @Override
    protected void end() {
        super.end();
        if (_notifier!=null) {
            _notifier.stop();
            _notifier.close();
            _notifier = null;
        }
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

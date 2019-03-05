package org.frc5687.deepspace.robot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDOutput;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;

import static  org.frc5687.deepspace.robot.Constants.Auto.AutoDrivePath.*;

public class AutoDrivePath extends OutliersCommand {
    private double _distance;
    private double _speed;
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;

    private DriveTrain _driveTrain;
    private AHRS _imu;

    private int _index = 0;

    private Notifier _notifier;
    private double _angleFactor;

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, double distance, double speed) {

        _driveTrain = driveTrain;
        requires(_driveTrain);
        _speed = speed;
        _imu = imu;
        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, 0, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
                new Waypoint(distance, 0, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        };
        info("Generating trajectory from 0,0,0 to " + distance + ",0,0 with dt=" + (1.0/ Constants.CYCLES_PER_SECOND) + ",  ");
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 1.0 / Constants.CYCLES_PER_SECOND, Constants.DriveTrain.CAP_SPEED_IPS, Constants.DriveTrain.MAX_ACCELERATION_IPSS, Constants.DriveTrain.MAX_JERK_IPSSS);
        _leftTrajectory = Pathfinder.generate(points, config);
        _rightTrajectory = Pathfinder.generate(points, config);

        info(_leftTrajectory.length() + " segments.");
/*        for (int i = 0; i < _trajectory.length(); i++) {
            Trajectory.Segment s= _trajectory.get(i);
            DriverStation.reportError("Seg " + i + " x=" + s.x + ", pos=" + s.position + ", vel=" + s.velocity + ", acc="+s.acceleration,false);
        }
*/
    }

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, String pathName) {
        _driveTrain = driveTrain;
        requires(_driveTrain);
        _imu = imu;
        _leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");
        _rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");

        info(_leftTrajectory.length() + " segments.");
    }

    @Override
    protected void initialize() {
        _driveTrain.resetDriveEncoders();
        _driveTrain.enableBrakeMode();
        _leftFollower = new DistanceFollower(_leftTrajectory);
        _rightFollower = new DistanceFollower(_rightTrajectory);
        _leftFollower.configurePIDVA(0.1, 0.0, 0.05, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);
        _rightFollower.configurePIDVA(0.1, 0.0, 0.05, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);

        _index = 0;

        _notifier = new Notifier(this::followPath);
        _notifier.startPeriodic(_leftTrajectory.get(0).dt);

    }

    @Override
    protected void execute() {
        _index++;



    }

    private void followPath() {
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            _notifier.stop();
        } else {
            _index++;
            double leftDistance = _driveTrain.getLeftDistance() / 12;
            double rightDistance = _driveTrain.getRightDistance() / 12;

            info("Left Segment " + _index + " target: " + _leftFollower.getSegment().x + " actual " + leftDistance + " vel=" + _leftFollower.getSegment().velocity);
            info("Right Segment " + _index + " target: " + _rightFollower.getSegment().x + " actual " + rightDistance + " vel=" + _rightFollower.getSegment().velocity);
            double leftSpeed = _leftFollower.calculate(leftDistance);
            double rightSpeed = _rightFollower.calculate(rightDistance);


            double heading = _imu.getYaw();
            double desired_heading = Pathfinder.r2d(_leftFollower.getHeading());
            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
            double turn =  0.8 * (-1.0/80.0) * heading_difference;
            _driveTrain.setPower(leftSpeed, rightSpeed, true);


            metric("Index", _index);
            metric("Left/Segment/X", _leftFollower.getSegment().x);
            metric("Left/Segment/Y", _leftFollower.getSegment().y);
            metric("Left/Segment/H", _leftFollower.getSegment().heading);
            metric("Left/Segment/V", _leftFollower.getSegment().velocity);
            metric("Left/Segment/D", _leftFollower.getSegment().position);
            metric("Left/Real/D", leftDistance);
            metric("Left/Real/H", _imu.getYaw());
            metric("Left/Real/V", leftSpeed -_angleFactor);

            metric("Right/Segment/X", _rightFollower.getSegment().x);
            metric("Right/Segment/Y", _rightFollower.getSegment().y);
            metric("Right/Segment/H", _rightFollower.getSegment().heading);
            metric("Right/Segment/V", _rightFollower.getSegment().velocity);
            metric("Right/Segment/D", _rightFollower.getSegment().position);
            metric("Right/Real/D", rightDistance);
            metric("Right/Real/H", _imu.getYaw());
            metric("Right/Real/V", rightSpeed +_angleFactor);

        }
    }
    @Override
    protected boolean isFinished() {
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            _notifier.stop();
            return true;
        };
        return  false;
    }

    @Override
    protected void end() {
        _driveTrain.setPower(0, 0, true);
    }
    private class PIDListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            _angleFactor = output;
        }

    }

}

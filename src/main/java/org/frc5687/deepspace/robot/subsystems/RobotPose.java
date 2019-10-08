package org.frc5687.deepspace.robot.subsystems;

import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.drive.TrackPose;
import org.frc5687.deepspace.robot.utils.sensors.Navx;

public class RobotPose extends OutliersSubsystem {

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Navx _navx;

    private double _x;
    private double _y;

    private double _heading;


    public RobotPose(Robot robot) {
        _robot = robot;
        _driveTrain = robot.getDriveTrain();
        _navx = new Navx();
    }

    @Override
    public void initDefaultCommand() { setDefaultCommand(new TrackPose(this, _driveTrain));
    }

    public void reset() {
        _navx.reset();
        _x = 0.0;
        _y = 0.0;
    }

    public void navxUpdate() {
        _navx.update();
    }


    public double getX() {
        return _x;
    }

    public void setX(double x) {
        _x = x;
    }

    public double getY() {
        return _y;
    }

    public void setY(double y) {
        _y = y;
    }

    public double getHeading() {
        return _heading;
    }

    public void setHeading(double heading) {
        _heading = heading;
    }

    @Override
    public void updateDashboard() {
    }
}

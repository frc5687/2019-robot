package org.frc5687.deepspace.robot.subsystems;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.utils.Limelight;
import org.frc5687.deepspace.robot.utils.TargetInfo;

import static org.frc5687.deepspace.robot.Constants.Positions.ANGLE_RANGE;


/**
 * Class to remember the robots position using localization
 */
public class RobotPose extends OutliersSubsystem{
    private Limelight _limelight;
    private double _xPosition;
    private double _yPosition;
    private double _heading;

    public RobotPose(Robot robot) {
        _limelight = robot.getLimelight();
    }

    @Override
    protected void initDefaultCommand() {
    }

    @Override
    public void updateDashboard() {
    }

    public void reset() {
        _xPosition = 0.0;
        _yPosition = 0.0;
        _heading = 0.0;
    }

    public double getXPosition() {
        return _xPosition;
    }

    public double getYPosition() {
        return _yPosition;
    }

    public void setXPosition(double x) {
        _xPosition = x;
    }

    public void setYPosition(double y) {
        _yPosition = y;
    }

    public double getHeading() {
        return _heading;
    }

    public void setHeading(double angle) {
        _heading =  angle;
    }

    public TargetInfo getTargetInfo()  {
        TargetInfo desiredTarget = new TargetInfo();

        desiredTarget.setX(_xPosition + _limelight.getTargetDistance() * Math.sin(Math.toRadians(_limelight.getHorizontalAngle()) + _heading));
        desiredTarget.setY(_yPosition + _limelight.getTargetDistance() * Math.cos(Math.toRadians(_limelight.getHorizontalAngle()) + _heading));

        return desiredTarget;
    }

    public boolean inRange(double angle) {
        return _heading >= angle - ANGLE_RANGE &&  _heading <= angle + ANGLE_RANGE;
    }








}

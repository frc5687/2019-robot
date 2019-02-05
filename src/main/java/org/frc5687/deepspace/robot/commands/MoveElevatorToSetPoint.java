package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class MoveElevatorToSetPoint extends OutliersCommand {

    private Elevator _elevator;
    private Elevator.Setpoint _setpoint;
    private Elevator.MotionMode _mode;
    private double _position = 0;
    public MoveElevatorToSetPoint(Elevator elevator, Elevator.Setpoint setpoint, Elevator.MotionMode mode) {
        _elevator = elevator;
        _setpoint = setpoint;
        _mode = mode;
    }

    @Override
    protected void initialize() {
        _position = _elevator.getPosition();
        info("Moving to setpoint " + _setpoint.name() + " (" + _setpoint.getValue() + ") using " + _mode.name() + " mode.");
        switch(_mode) {
            case Simple:

        }

    }

    @Override
    protected void execute() {
        _position = _elevator.getPosition();
        switch(_mode) {
            case Simple:
                if (_position > _setpoint.getValue() + Constants.Elevator.TOLERANCE) {
                    _elevator.setElevatorSpeeds(Constants.Elevator.SPEED_UP);
                } else if (_position < _setpoint.getValue() - Constants.Elevator.TOLERANCE) {
                    _elevator.setElevatorSpeeds(Constants.Elevator.SPEED_DOWN);
                } else {
                    _elevator.setElevatorSpeeds(0);
                }
                break;
            default:
                _elevator.setElevatorSpeeds(0);
        }
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(_position-_setpoint.getValue()) <= Constants.Elevator.TOLERANCE;
    }

    @Override
    protected void end() {
        info("Reached setpoint " + _setpoint.name());
        super.end();
    }
}

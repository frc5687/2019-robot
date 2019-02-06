package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class MoveElevatorToSetPoint extends OutliersCommand {

    private Elevator _elevator;
    private Elevator.Setpoint _setpoint;
    private Elevator.MotionMode _mode;
    private double _position = 0;

    private double _pidOutput;

    private PIDController _pidController;

    public MoveElevatorToSetPoint(Elevator elevator, Elevator.Setpoint setpoint, Elevator.MotionMode mode) {
        _elevator = elevator;
        _setpoint = setpoint;
        _mode = mode;

        _pidController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD, _elevator, new PIDListener());
        _pidController.setAbsoluteTolerance(Constants.Elevator.TOLERANCE);
        _pidController.setOutputRange(-Constants.Elevator.MAX_ELEVATOR_SPEED_DOWN, Constants.Elevator.MAX_ELEVATOR_SPEED_UP);
        _pidController.setInputRange(Elevator.Setpoint.Bottom.getValue(), Elevator.Setpoint.Top.getValue());
        _pidController.disable();
    }

    @Override
    protected void initialize() {
        _position = _elevator.getPosition();
        info("Moving to setpoint " + _setpoint.name() + " (" + _setpoint.getValue() + ") using " + _mode.name() + " mode.");
        switch(_mode) {
            case Simple:
                break;
            case PID:
                _pidController.setSetpoint(_setpoint.getValue());
                _pidController.enable();
                break;
        }

    }

    @Override
    protected void execute() {
        _position = _elevator.getPosition();
        switch(_mode) {
            case Simple:
                if (_position  < _setpoint.getValue() - Constants.Elevator.TOLERANCE) {
                    _elevator.setElevatorSpeeds(Constants.Elevator.SPEED_UP);
                } else if (_position > _setpoint.getValue() + Constants.Elevator.TOLERANCE) {
                    _elevator.setElevatorSpeeds(-Constants.Elevator.SPEED_DOWN);
                } else {
                    _elevator.setElevatorSpeeds(0);
                }
                break;
            case PID:
                _elevator.setElevatorSpeeds(_pidOutput);
                break;
            default:
                _elevator.setElevatorSpeeds(0);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(_position-_setpoint.getValue()) <= Constants.Elevator.TOLERANCE;
    }

    @Override
    protected void end() {
        _pidController.disable();
        _elevator.setElevatorSpeeds(0);
        info("Reached setpoint " + _setpoint.name() + " (" + _position + ")");
    }

    private class PIDListener implements PIDOutput {

        public double get() {
            return _pidOutput;
        }

        @Override
        public void pidWrite(double output) {
            _pidOutput = output;
        }

    }

}

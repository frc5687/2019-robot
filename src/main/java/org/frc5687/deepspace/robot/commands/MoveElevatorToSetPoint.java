package org.frc5687.deepspace.robot.commands;

import com.sun.xml.internal.bind.v2.runtime.reflect.opt.Const;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Elevator;

import static org.frc5687.deepspace.robot.Constants.Elevator.*;
import static org.frc5687.deepspace.robot.subsystems.Elevator.RampingMode.Down;
import static org.frc5687.deepspace.robot.subsystems.Elevator.RampingMode.Ramp;
import static org.frc5687.deepspace.robot.subsystems.Elevator.RampingMode.Steady;

public class MoveElevatorToSetPoint extends OutliersCommand {

    private Elevator _elevator;
    private Elevator.Setpoint _setpoint;
    private Elevator.MotionMode _mode;
    private Elevator.RampingMode _rampingMode;
    private double _position = 0;

    private double _pidOutput;
    private double _pathOutput;

    private PIDController _pidController;

    private Trajectory _trajectory;
    private DistanceFollower _pathFollower;
    private Notifier _pathNotifier;
    private long _startTime;
    private double _step;

    public MoveElevatorToSetPoint(Elevator elevator, Elevator.Setpoint setpoint, Elevator.MotionMode mode) {
        _elevator = elevator;
        requires(_elevator);
        _setpoint = setpoint;
        _mode = mode;

        _pidController = new PIDController(PID.kP, PID.kI, PID.kD, _elevator, new PIDListener());
        _pidController.setAbsoluteTolerance(TOLERANCE);
        _pidController.setOutputRange(-MAX_ELEVATOR_SPEED_DOWN, MAX_ELEVATOR_SPEED_UP);
        _pidController.setInputRange(Elevator.Setpoint.Bottom.getValue(), Elevator.Setpoint.Top.getValue());
        _pidController.disable();
    }

    @Override
    protected void initialize() {
        _step = 0;
        _position = _elevator.getPosition();
        if (withinTolerance()) { return; }
        info("Moving to setpoint " + _setpoint.name() + " (" + _setpoint.getValue() + ") using " + _mode.name() + " mode.");
        switch(_mode) {
            case Simple:
                break;
            case PID:
                _pidController.setSetpoint(_setpoint.getValue());
                _pidController.enable();
                break;
            case Path:
                _trajectory = getTrajectory((long)_elevator.getPosition(), _setpoint.getValue());
                _pathFollower = new DistanceFollower(_trajectory);
                _pathFollower.configurePIDVA(Path.kP, Path.kI, Path.kD, 1/MAX_VELOCITY_IPS, 0);

                _pathNotifier = new Notifier(this::followPath);
                _pathNotifier.startPeriodic(_trajectory.get(0).dt);
                break;
        }
        _startTime = System.currentTimeMillis();
    }

    @Override
    protected void execute() {
        _step++;
        double speed;

        _position = _elevator.getPosition();
        switch(_mode) {
            case Simple:
                if (_position  < _setpoint.getValue() - TOLERANCE) {
                    _elevator.setSpeed(SPEED_UP);
                } else if (_position > _setpoint.getValue() + TOLERANCE) {
                    _elevator.setSpeed(-SPEED_DOWN);
                } else {
                    _elevator.setSpeed(0);
                }
                break;
            case PID:
                _elevator.setSpeed(_pidOutput);
                break;
            case Path:
                _elevator.setSpeed(_pathOutput);
                break;
            case Ramp:
                if (_position  < _setpoint.getValue() - TOLERANCE) {
                    _elevator.setElevatorSpeeds(getRampedSpeed(SPEED_UP));
                } else if (_position > _setpoint.getValue() + TOLERANCE) {
                    _elevator.setElevatorSpeeds(getRampedSpeed(-SPEED_DOWN));
                } else {
                    _elevator.setElevatorSpeeds(0);
                }
                break;
            default:
                _elevator.setSpeed(0);
                break;
        }
    }

    private double getRampedSpeed(double speed) {
        if (speed < 0) { return speed; }
        switch(_rampingMode) {
            case Ramp:
                speed = (Constants.Elevator.MIN_SPEED +(_step/Constants.Elevator.STEPS))* (Constants.Elevator.GOAL_SPEED - Constants.Elevator.MIN_SPEED);
                _elevator.setElevatorSpeeds(speed);
                if (speed == Constants.Elevator.GOAL_SPEED) {
                    _elevator.setRampingMode(Steady);
                }
            case Steady:
                _elevator.setElevatorSpeeds(Constants.Elevator.GOAL_SPEED);
                if(_setpoint.getValue() == _elevator.getRawMAGEncoder() - 200) {
                    _step = 0;
                    _elevator.setRampingMode(Down);
                }
            case Down:
                speed = (Constants.Elevator.GOAL_SPEED -(_step/Constants.Elevator.STEPS))* (Constants.Elevator.GOAL_SPEED - Constants.Elevator.MIN_SPEED);
                _elevator.setElevatorSpeeds(speed);
                if (_elevator.getRawMAGEncoder() == _setpoint.getValue()) {

                }
        }
        return speed;
    }
    private boolean withinTolerance() {
        return Math.abs(_position-_setpoint.getValue()) <= TOLERANCE;
    }
    @Override
    protected boolean isFinished() {
        if (withinTolerance()) {
            return true;
        };
        switch (_mode) {
            case PID:
                return _pidController.onTarget();
            case Path:
                return _pathFollower.isFinished();
        }
        return false;
    }

    @Override
    protected void end() {
        long endTime = System.currentTimeMillis();
        DriverStation.reportError("MoveElevatorToSetpoint Ran for " + (endTime - _startTime) + " millis", false);
        if (_pidController!=null) {
            _pidController.disable();
        }
        if (_pathNotifier!=null) {
            _pathNotifier.stop();
        }
        _elevator.setSpeed(0);
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

    private void followPath() {
        if (_pathFollower.isFinished()) {
            _pathNotifier.stop();
        } else {
            double _speed = _pathFollower.calculate(_elevator.getPosition());
            _pathOutput = _speed;
        }
    }


    private Trajectory getTrajectory(long startPosition, long endPosition) {
        double startInches = startPosition/Constants.Elevator.TICKS_PER_INCH;
        double endInches = endPosition/Constants.Elevator.TICKS_PER_INCH;

        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, startInches, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
                new Waypoint(0, endInches, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        };

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 0.02, MAX_VELOCITY_IPS, MAX_VELOCITY_IPS, 60.0);
        Trajectory trajectory = Pathfinder.generate(points, config);

        return trajectory;
    }
}

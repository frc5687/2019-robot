package org.frc5687.deepspace.robot.commands;

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

public class MoveElevatorToSetPoint extends OutliersCommand {

    private Elevator _elevator;
    private Elevator.Setpoint _setpoint;
    private Elevator.MotionMode _mode;
    private RampingMode _rampingMode;
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
        _pidController.setOutputRange(-MAX_SPEED_DOWN, MAX_SPEED_UP);
        _pidController.setInputRange(Elevator.Setpoint.Bottom.getValue(), Elevator.Setpoint.Top.getValue());
        _pidController.disable();
    }

    @Override
    protected void initialize() {
        _step = 0;
        _position = _elevator.getPosition();
        if (withinTolerance()) { return; }
        DriverStation.reportError("Moving to setpoint " + _setpoint.name() + " (" + _setpoint.getValue() + ") using " + _mode.name() + " mode.", false);
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
            case Ramp:
                _rampingMode = RampingMode.Ramp;
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
                _elevator.setSpeed(_pidOutput, true);
                break;
            case Path:
                _elevator.setSpeed(_pathOutput);
                break;
            case Ramp:
                if (_position  < _setpoint.getValue() - TOLERANCE) {
                    _elevator.setSpeed(getRampedSpeed(SPEED_UP));
                } else if (_position > _setpoint.getValue() + TOLERANCE) {
                    _elevator.setSpeed(getRampedSpeed(-SPEED_DOWN));
                } else {
                    _elevator.setSpeed(0);
                }
                break;
            default:
                _elevator.setSpeed(0);
                break;
        }
    }

    private double getRampedSpeed(double speed) {
        metric("Ramp/RawSpeed", speed);
        metric("Ramp/Mode", _rampingMode.name());
        metric("Ramp/Step", _step);

        switch(_rampingMode) {
            case Ramp:
                speed = MIN_SPEED + _step * ((GOAL_SPEED - MIN_SPEED) / STEPS_UP);
                if(Math.abs(_setpoint.getValue() - _elevator.getPosition()) <=  TICKS_PER_STEP * STEPS_DOWN) {
                    _step = 0;
                    _rampingMode = RampingMode.Down;
                } else if (_step >= STEPS_UP) {
                    _rampingMode = RampingMode.Steady;
                }
                break;
            case Steady:
                if(Math.abs(_setpoint.getValue() - _elevator.getPosition()) <=  TICKS_PER_STEP * STEPS_DOWN) {
                    _step = 0;
                    _rampingMode = RampingMode.Down;
                }
                break;
            case Down:
                speed = MIN_SPEED + (STEPS_DOWN - _step) * ((GOAL_SPEED - MIN_SPEED) / STEPS_DOWN);
                //speed = (Constants.Elevator.GOAL_SPEED -(_step/Constants.Elevator.STEPS))* (Constants.Elevator.GOAL_SPEED - Constants.Elevator.MIN_SPEED);
                //if (_elevator.getRawMAGEncoder() == _setpoint.getValue()) {
                //}
        }

        metric("Ramp/RampedSpeed", speed);

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


    private enum RampingMode {
        Ramp(0),
        Steady(1),
        Down(2);

        private int _value;

        RampingMode(int value) { this._value = value; }

        public int getValue() { return _value; }
    }

}

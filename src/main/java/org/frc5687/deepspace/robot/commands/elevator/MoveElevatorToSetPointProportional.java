package org.frc5687.deepspace.robot.commands.elevator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.Elevator;
import org.frc5687.deepspace.robot.utils.Helpers;

import static org.frc5687.deepspace.robot.Constants.Elevator.*;
import static org.frc5687.deepspace.robot.utils.Helpers.limit;

/***
 * Command to move the elevator to a setpoint using Spark ramping to start moving and proportional PID
 */
public class MoveElevatorToSetPointProportional extends OutliersCommand {

    private Elevator _elevator;
    private Elevator.Setpoint _setpoint;
    private double _position = 0;

    private OI _oi;

    private Notifier _notifier;
    private long _startTime;
    private int _initialDirection = 0;
    private double _speed;
    private double kP = (MAX_SPEED - MIN_SPEED) / RAMP_TICKS;


    public MoveElevatorToSetPointProportional(Elevator elevator, Elevator.Setpoint setpoint, OI oi, double speed) {
        _elevator = elevator;
        requires(_elevator);
        _setpoint = setpoint;
        _speed = speed;
        _notifier = new Notifier(this::run);
        _oi = oi;
    }

    @Override
    protected void initialize() {
        super.initialize();
        _position = _elevator.getPosition();
        _initialDirection = (int)Math.copySign(1, _setpoint.getValue() - _position);

        _elevator.enableSparkRamping(Constants.Elevator.SPARK_RAMPING);
        if (_setpoint== Elevator.Setpoint.ClearBumper && _position > Elevator.Setpoint.ClearBumper.getValue()) {
            error("Skipping setpoint " + _setpoint.name() + "  b/c height is " + _position);
            return;
        }

        if (withinTolerance()) { return; }
        error("Moving to setpoint " + _setpoint.name() + " (" + _setpoint.getValue() + ") using proportional mode.");

        _notifier.startPeriodic(0.05);
        _startTime = System.currentTimeMillis();
    }

    protected void run() {
        _position = _elevator.getPosition();
        double _distance = _setpoint.getValue() - _position;
        double speed = Helpers.limit(_distance * kP, -1, 1);
        speed  += Math.copySign(1, _distance) * MIN_SPEED;

                // Math.copySign(Helpers.limit(Math.abs(speed), 0.2, 1), speed);
        _elevator.setSpeed(speed);
    }

    @Override
    protected void execute() {
        // Don't do anything here since we do the work in run()

        metric("Position", _position);
        metric("Setpoint", _setpoint.getValue());
        metric("TopHall", _elevator.isAtTop());
        metric("BottomHall", _elevator.isAtBottom());
    }

    private boolean withinTolerance() {
        return Math.abs(_setpoint.getValue() - _position) <= TOLERANCE;
    }

    @Override
    protected boolean isFinished() {
        if (_setpoint== Elevator.Setpoint.ClearBumper && _position > Elevator.Setpoint.ClearBumper.getValue()) {
            error("Skipping setpoint " + _setpoint.name() + "  b/c height is " + _position);
            return true;
        }

        if (withinTolerance()) {
            return true;
        }
        return false;
    }

    @Override
    protected void end() {
        if (_notifier!=null) {
            _notifier.stop();
        }

        long endTime = System.currentTimeMillis();
        _elevator.setSpeed(0);

        if (_oi!=null) {
            _oi.pulseOperator(1);
        }

        info("Reached setpoint " + _setpoint.name() + " (" + _position + ") + in " + (endTime - _startTime) + " millis." );
        super.end();
    }
}

package org.frc5687.deepspace.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Arm;

import static org.frc5687.deepspace.robot.Constants.Arm.*;

public class MoveArmToSetPoint extends OutliersCommand {
    private Arm _arm;
    private Arm.Setpoint _setpoint;
    private Arm.HallEffectSensor _hallEffectSensor;
    private Arm.MotionMode _mode;
    private double _position = 0;

    private int _direction = 0;

    private double _pidOutput;

    private long _startTime;
    private PIDController _pidController;

    public MoveArmToSetPoint(Arm arm, Arm.Setpoint setpoint, Arm.HallEffectSensor hallEffectSensor, Arm.MotionMode mode){
        _arm = arm;
        _hallEffectSensor = hallEffectSensor;
        _setpoint = setpoint;
        _mode = mode;
        _hallEffectSensor = hallEffectSensor;
        requires(_arm);
    }

    @Override
    protected void initialize(){
        _position = _arm.getPosition();
        if (withinTolerance()) { return; }
        info("Moving to setpoint " + _setpoint.name() + " (" + _setpoint.getValue() + ") using " + _mode.name() + " mode.");
        switch(_mode) {
            case HallOnly:
                _direction = getDirection(_hallEffectSensor);
                break;
            case Simple:
                break;
            case PID:
                _pidController.setSetpoint(_setpoint.getPosition());
                _pidController.enable();
                break;
        }
        _startTime = System.currentTimeMillis();
   }

    @Override
    protected void execute() {
        _position = _arm.getPosition();
        switch(_mode) {
            case HallOnly:
                _arm.setSpeed(_direction * SPEED);
                break;
            case Simple:
                if (_position  < _setpoint.getPosition() - TOLERANCE) {
                    _arm.setSpeed(SPEED_UP);
                } else if (_position > _setpoint.getPosition() + TOLERANCE) {
                    _arm.setSpeed(-SPEED_DOWN);
                } else {
                    _arm.setSpeed(0);
                }
                break;
            case PID:
                _arm.setSpeed(_pidOutput);
                break;
            default:
                _arm.setSpeed(0);
                break;
        }
    }

    private int getDirection(Arm.HallEffectSensor hallEffectSensor) {
        switch (hallEffectSensor) {
            case LOW:
                if (_arm.isLow()) {return 0;}
                return 1;
            case INTAKE:
                if (_arm.isLow()) {return -1;}
                if (_arm.isIntake()) {return 0;}
                return 1;
            case SECURE:
                if (_arm.isStowed()) {return 1;}
                if (_arm.isSecured()) {return 0;}
                return -1;
            case STOWED:
                if (_arm.isStowed()) {return 0;}
                return -1;
        }
        return 0;
    }


    private boolean withinTolerance() {
        return Math.abs(_position-_setpoint.getPosition()) <= TOLERANCE;
    }

    @Override
    protected boolean isFinished() {
        switch (_hallEffectSensor){
            case LOW:
                if(_arm.isLow()){
                    return true;
                }
                break;
            case INTAKE:
                if(_arm.isIntake()){
                    return true;
                }
                break;
            case SECURE:
                if(_arm.isSecured()){
                    return true;
                }
                break;
            case STOWED:
                if(_arm.isStowed()){
                    return true;
                }
                break;
        }
        if (withinTolerance()) {
            return true;
        };
        switch(_mode) {
            case PID:
                return _pidController.onTarget();
            default:
                return false;
        }
    }

    @Override
    protected void end(){
        long endTime = System.currentTimeMillis();
        DriverStation.reportError("Ran for " + (endTime - _startTime) + " millis", false);
        if (_pidController!=null) {
            _pidController.disable();
        }
        _arm.setSpeed(0);
        info("Reached setpoint " + _setpoint.name() + " (" + _position + ")");
    }


}

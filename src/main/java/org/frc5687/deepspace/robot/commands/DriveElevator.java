package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class DriveElevator extends OutliersCommand{
    
    private Elevator _elevator;
    private OI _oi;

    public DriveElevator(Robot robot, Elevator elevator) {
        _elevator = elevator;
        _oi = robot.getOI();
        requires(_elevator);
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        // Read speed stick positions from OI
        double speed = _oi.getElevatorSpeed();
        if (_elevator.isNearBottom()) {
            speed = _oi.getElevatorSpeed();
        } else if (Math.abs(speed) <= Constants.Elevator.DEADBAND) {
            speed = Constants.Elevator.HOLD_SPEED;
        }
        // Send to the Elevator
        _elevator.setSpeed(speed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        // Set Elevator motor speeds to 0 and set break mode?s
    }
}


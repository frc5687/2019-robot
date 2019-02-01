package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class DriveElevator extends OutliersCommand{
    
    private Elevator _Elevator;
    private OI _oi;

    public DriveElevator(Robot robot, Elevator Elevator) {
        _Elevator = Elevator;
        _oi = robot.getOI();
        requires(_Elevator);
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        // Read speed stick positions from OI
        double speed = _oi.getElevatorSpeed();

        // Send to the Elevator
        _Elevator.setSpeeds(speed);
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


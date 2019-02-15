package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Robot;

public class CargoModeSet extends OutliersCommand {

    private Robot _robot;
    public CargoModeSet(Robot robot) {
        _robot = robot;

    }

    @Override
    protected void initialize() {
        _robot.setConfiguration(Robot.Configuration.cargo);
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



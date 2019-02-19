package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Robot;

public class SetConfiguration extends OutliersCommand {

    private Robot _robot;
    private Robot.Configuration _configuration;

    public SetConfiguration(Robot robot, Robot.Configuration configuration) {
        _robot = robot;
        _configuration = configuration;
    }

    @Override
    protected void initialize() {
        _robot.setConfiguration(_configuration);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }
}



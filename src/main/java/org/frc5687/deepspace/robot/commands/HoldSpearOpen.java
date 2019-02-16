package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Spear;

public class HoldSpearOpen extends OutliersCommand {
    private Robot _robot;
    private Spear _spear;

    public HoldSpearOpen(Robot robot) {
        _robot = robot;
        _spear = robot.getSpear();
        requires(_spear);
    }
    @Override
    protected void initialize() {
        if (_robot.getConfiguration() == Robot.Configuration.hatch) {
            _spear.open();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        if (_robot.getConfiguration() == Robot.Configuration.hatch) {
            _spear.close();
        }
    }

    @Override
    protected void execute(){
    }
}

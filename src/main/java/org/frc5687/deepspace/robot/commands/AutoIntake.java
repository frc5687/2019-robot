package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Spear;

public class AutoIntake extends OutliersCommand {

    private Robot _robot;


    public AutoIntake(Robot robot) {
        _robot = robot;
    }

    @Override
    protected void initialize() {
        if (_robot.getConfiguration() == Robot.Configuration.hatch) {
            (new OpenSpear(_robot.getIntake())).start();
        } else if(_robot.getConfiguration() == Robot.Configuration.cargo) {
            (new CargoIntake(_robot)).start();
        }
        metric("Mode", _robot.getConfiguration().toString());
    }

    @Override
    public void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}


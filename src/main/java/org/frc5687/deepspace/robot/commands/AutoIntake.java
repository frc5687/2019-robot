package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;

public class AutoIntake extends OutliersCommand {

    private Robot _robot;


    public AutoIntake(Robot robot) {
        _robot = robot;
    }

    @Override
    protected void initialize() {
        if (_robot.getConfiguration() == Robot.Configuration.hatch) {
            (new GripClaw(_robot.getHatchIntake())).start();
        } else if(_robot.getConfiguration() == Robot.Configuration.cargo) {
            (new IntakeCargo(_robot)).start();
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


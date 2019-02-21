package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Intake;
import org.frc5687.deepspace.robot.subsystems.Spear;

public class HoldSpearOpen extends OutliersCommand {
    private Robot _robot;
    private Intake _intake;

    public HoldSpearOpen(Robot robot) {
        _robot = robot;
       // _intake = robot.getIntake();
        requires(_intake);
    }
    @Override
    protected void initialize() {
//        if (_robot.getConfiguration() == Robot.Configuration.hatch) {
            _intake.openTalons();
//        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
//        if (_robot.getConfiguration() == Robot.Configuration.hatch) {
            _intake.closeTalons();
//        }
    }

    @Override
    protected void execute(){
    }
}

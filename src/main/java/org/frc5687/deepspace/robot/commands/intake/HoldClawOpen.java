package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.Intake;

public class HoldClawOpen extends OutliersCommand {
    private Robot _robot;
    private Intake _intake;

    public HoldClawOpen(Robot robot) {
        _robot = robot;
        _intake = robot.getIntake();
        requires(_intake);
    }
    @Override
    protected void initialize() {
        _intake.gripClaw();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
            _intake.pointClaw();
    }

    @Override
    protected void execute(){
    }
}

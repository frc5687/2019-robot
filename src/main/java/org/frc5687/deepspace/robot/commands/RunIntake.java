package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Intake;

public class RunIntake extends OutliersCommand {

    private Intake _intake;
    private OI _oi;

    public RunIntake (Robot robot, Intake intake) {
        _intake = intake;
        _oi = robot.getOI();
        requires(_intake);

    }
    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        double speed = _oi.getRollerSpeed();
        _intake.setSpeed(speed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

}

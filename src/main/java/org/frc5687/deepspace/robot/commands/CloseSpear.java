package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Intake;
import org.frc5687.deepspace.robot.subsystems.Spear;
public class CloseSpear extends OutliersCommand {
    public Intake _intake;
    private long _startTime;

    public CloseSpear(Intake intake) {
        _intake = intake;
        requires(_intake);
    }
    @Override
    protected void initialize() {
        _intake.closeTalons();
        _startTime = System.currentTimeMillis();
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Spear.CLOSE_MILLI_SEC;
    }

    @Override
    protected void execute(){
    }
}

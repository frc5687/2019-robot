package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Intake;
import org.frc5687.deepspace.robot.subsystems.Spear;
public class OpenSpear extends OutliersCommand {
    private Intake _intake;
    private long _startTime;

    public OpenSpear(Intake intake) {
        _intake = intake;
        requires(_intake);
    }
    @Override
    protected void initialize() {
        _intake.openTalons();
        _startTime = System.currentTimeMillis();
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Spear.OPEN_MILLI_SEC;
    }

    @Override
    protected void execute(){
    }
}

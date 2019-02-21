package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Intake;

public class GripClaw extends OutliersCommand {
    private Intake _intake;
    private long _startTime;

    public GripClaw(Intake intake) {
        _intake = intake;
        requires(_intake);
    }
    @Override
    protected void initialize() {
        _intake.gripClaw();
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

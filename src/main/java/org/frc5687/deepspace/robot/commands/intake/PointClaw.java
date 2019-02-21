package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.Intake;

public class PointClaw extends OutliersCommand {
    private Intake _intake;
    private long _startTime;

    public PointClaw(Intake intake) {
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
        return System.currentTimeMillis() > _startTime + Constants.Intake.OPEN_CLAW_MILLI_SEC;
    }

    @Override
    protected void execute(){
    }
}

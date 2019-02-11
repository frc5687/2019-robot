package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Wrist;

public class WristDown extends OutliersCommand {
    public Wrist _wrist;
    private boolean _done = false;
    private long _startTime;

    public WristDown(Wrist wrist) {
        _wrist = wrist;
        requires(_wrist);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Wrist.LOWER_MILLI_SEC;
    }
    @Override
    protected void initialize() {
        _done = false;
        _startTime = System.currentTimeMillis();
    }
    @Override
    protected void execute(){
        _wrist.Down();
        _done = true;
    }
}

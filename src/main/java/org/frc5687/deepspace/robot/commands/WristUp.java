package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Wrist;

public class WristUp extends OutliersCommand {
    public Wrist _wrist;
    private boolean _done = false;
    private long _startTime;

    public WristUp(Wrist wrist) {
        _wrist = wrist;
        requires(_wrist);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Wrist.RAISE_MILLI_SEC;
    }
    @Override
    protected void initialize() {
        _done = false;
        _startTime = System.currentTimeMillis();
    }
    @Override
    protected void execute(){
        _wrist.Up();
        _done = true;
    }
}

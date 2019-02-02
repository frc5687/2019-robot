package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Wrist;

public class WristDown extends OutliersCommand {
    public Wrist _wrist;
    private boolean _done = false;

    public WristDown(Wrist wrist) {
        _wrist = wrist;
        requires(_wrist);
    }
    @Override
    protected boolean isFinished() {
        return false;
    }
    @Override
    protected void initialize() {
        _done = false;
    }
    @Override
    protected void execute(){
        _wrist.Down();
        _done = true;
    }
}

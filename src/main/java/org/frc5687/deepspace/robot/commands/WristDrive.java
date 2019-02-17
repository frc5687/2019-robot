package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Wrist;

public class WristDrive extends OutliersCommand {
    private Wrist _wrist;
    private boolean _done = false;
    public WristDrive(Wrist wrist){
        _wrist = wrist;
        requires(_wrist);

        //Don't know what to put here, if anything.
    }

    @Override
    protected void initialize() {
        _done = false;
    }
    @Override
    protected void execute(){
        isFinished();
    }
    @Override
    protected boolean isFinished() {
        return _done;
    }
}
package org.frc5687.deepspace.robot.commands;

public class WristDrive extends OutliersCommand {

    private boolean _done = false;
    public WristDrive(){
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
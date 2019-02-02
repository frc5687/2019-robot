package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Gripper;

public class SuckBall extends OutliersCommand {
    public Gripper _gripper;
    private boolean _done = false;

    public SuckBall(Gripper gripper){
        _gripper = gripper;
        requires(_gripper);
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
        _gripper.suckBall();
        _done = true;
    }
}

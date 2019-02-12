package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Gripper;

public class StartGripper extends OutliersCommand {
    public Gripper _gripper;

    public StartGripper(Gripper gripper){
        _gripper = gripper;
        requires(_gripper);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void initialize() {
        _gripper.start();
    }

    @Override
    protected void execute(){
        _gripper.start();
    }

}

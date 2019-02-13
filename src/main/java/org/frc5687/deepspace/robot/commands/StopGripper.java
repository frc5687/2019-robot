package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Gripper;

public class StopGripper extends OutliersCommand {
    public Gripper _gripper;

    public StopGripper(Gripper gripper){
        _gripper = gripper;
        requires(_gripper);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void initialize() {
        _gripper.stop();
    }

    @Override
    protected void execute(){
        _gripper.stop();
    }

}

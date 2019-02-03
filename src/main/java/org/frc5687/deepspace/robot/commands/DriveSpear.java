package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Spear;

public class DriveSpear extends OutliersCommand {
    Spear _spear;
    public DriveSpear(Spear spear){
        _spear = spear;
        requires(_spear);

        //Don't know what to put here, if anything.
    }

    @Override
    protected void initialize() {

    }
    @Override
    protected void execute(){
    }
    @Override
    protected boolean isFinished() {
        return false;
    }
}

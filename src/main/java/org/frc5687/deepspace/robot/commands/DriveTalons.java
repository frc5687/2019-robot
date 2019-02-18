package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Talons;

public class DriveTalons extends OutliersCommand {
    Talons _talons;

    public DriveTalons(Talons talons){
        _talons = talons;
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

package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Talons;

public class CloseTalons extends OutliersCommand {
    private Talons _talons;

    public CloseTalons(Talons talons) {
        _talons = talons;
        requires(talons);
    }

    @Override
    protected void initialize() {
        _talons.open();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }


    @Override
    protected void execute(){
    }



}


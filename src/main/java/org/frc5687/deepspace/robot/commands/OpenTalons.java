package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Talons;
import org.frc5687.deepspace.robot.Robot;

public class OpenTalons extends OutliersCommand {
    private Talons _talons;

    public OpenTalons(Talons talons) {
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

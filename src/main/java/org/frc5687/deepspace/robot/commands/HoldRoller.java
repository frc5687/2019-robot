package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Roller;

public class HoldRoller extends OutliersCommand {

    private Roller _roller;
    private OI _oi;

    public HoldRoller(Roller roller) {
        _roller = roller;
        requires(_roller);
    }
    @Override
    protected void execute() {
        _roller.run(0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

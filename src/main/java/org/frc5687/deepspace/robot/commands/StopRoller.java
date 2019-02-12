package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Roller;

public class StopRoller extends OutliersCommand {
    private Roller _roller;
    private OI _oi;

    public StopRoller(Roller roller) {
        _roller = roller;
        requires(_roller);
    }

    @Override
    protected void execute() {
        _roller.stop();
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}

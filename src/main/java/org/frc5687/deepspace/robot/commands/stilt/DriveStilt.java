package org.frc5687.deepspace.robot.commands.stilt;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.Stilt;

public class RunStilt extends OutliersCommand {
    private Stilt _stilt;
    private OI _oi;

    public RunStilt(Stilt stilt, OI oi) {
        _stilt = stilt;
        _oi = oi;
        requires(_stilt);
    }

    @Override
    protected void execute() {
        // Get value from OI
        double speed = _oi.getStiltSpeed();

        // write to stilt
        _stilt.drive(speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

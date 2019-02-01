package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Gobbler;

public class GobblerIntake extends OutliersCommand {

    private Gobbler _gobbler;
    private OI _oi;

    public GobblerIntake(Gobbler gobbler) {
        _gobbler = gobbler;
        requires(_gobbler);
    }
    @Override
    protected void execute() {
        _gobbler.setIntakeState(Gobbler.IntakeState.INTAKE);
        _gobbler.runIntake(0);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}

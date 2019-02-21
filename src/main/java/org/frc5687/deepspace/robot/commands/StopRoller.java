package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Intake;
import org.frc5687.deepspace.robot.subsystems.Roller;

public class StopRoller extends OutliersCommand {
    private Roller _roller;
    private Intake _intake;
    private OI _oi;

    public StopRoller(Intake intake) {
        _intake = intake;
        requires(_intake);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        _intake.stopRoller();
    }



    @Override
    protected boolean isFinished() {
        return true;
    }
}

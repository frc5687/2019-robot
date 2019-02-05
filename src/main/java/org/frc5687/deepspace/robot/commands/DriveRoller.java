package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.Roller;

public class DriveRoller extends OutliersCommand {

    private Roller _roller;
    private OI _oi;

    public DriveRoller(Roller roller) {
        _roller = roller;
        requires(_roller);
    }
    @Override
    protected void execute() {
        _roller.run(Constants.Roller.MAX_SPEED);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Spear;
public class OpenSpear extends OutliersCommand {
    private Spear _spear;
    private long _startTime;

    public OpenSpear(Spear spear) {
        _spear = spear;
        requires(_spear);
    }
    @Override
    protected void initialize() {
        _spear.open();
        _startTime = System.currentTimeMillis();
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Spear.OPEN_MILLI_SEC;
    }

    @Override
    protected void execute(){
    }
}

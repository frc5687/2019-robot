package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Spear;
public class CloseSpear extends OutliersCommand {
    public Spear _spear;
    private boolean _done = false;
    private long _startTime;

    public CloseSpear(Spear spear) {
        _spear = spear;
        requires(_spear);
    }
    @Override
    protected void initialize() {
        _done = false;
        _startTime = System.currentTimeMillis();
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Spear.CLOSE_MILLI_SEC;
    }

    @Override
    protected void execute(){
        _spear.close();
        _done = true;
    }
}

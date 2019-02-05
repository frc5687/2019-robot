package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.subsystems.Spear;
public class OpenSpear extends OutliersCommand {
    public Spear _spear;
    private boolean _done = false;

    public OpenSpear(Spear spear) {
        _spear = spear;
        requires(_spear);
    }
    @Override
    protected void initialize() {
        _done = false;
    }

    @Override
    protected boolean isFinished() {
        return _done;
    }

    @Override
    protected void execute(){
       _spear.open();
       _done = true;
    }
}

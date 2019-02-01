package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Gobbler;

public class DriveGobbler extends OutliersCommand {

    private Gobbler _gobbler;
    private OI _oi;

    public DriveGobbler(Robot robot, Gobbler gobbler) {
        _gobbler = gobbler;
        _oi = robot.getOI();
        requires(_gobbler);
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        double speed = _oi.getGobblerSpeed();

        _gobbler.setSpeeds(speed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        // Set gobbler motor speeds to 0 and set break mode?s
    }
}


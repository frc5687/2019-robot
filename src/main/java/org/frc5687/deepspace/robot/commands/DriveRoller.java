package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.Roller;

public class DriveRoller extends OutliersCommand {

    private Roller _roller;
    private OI _oi;

    public DriveRoller(Robot robot, Roller roller) {
        _roller = roller;
        _oi = robot.getOI();
        requires(_roller);
    }
    @Override
    protected void execute() {
        double speed = _oi.getRollerSpeed();
        _roller.setRollerSpeed(speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

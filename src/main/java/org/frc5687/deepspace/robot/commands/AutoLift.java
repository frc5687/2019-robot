package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.Stilt;

public class AutoLift extends OutliersCommand {
    private Arm _arm;
    private Stilt _stilt;

    private double armSpeed = 0.9;
    private double stiltSpeed = 0.6;

    public AutoLift(Arm arm, Stilt stilt) {
        _arm = arm;
        _stilt = stilt;
        requires(_arm);
        requires(_stilt);
    }


    @Override
    protected void execute() {
        _arm.setSpeed(armSpeed);
        _stilt.drive(stiltSpeed);
    }

    @Override
    protected void end() {
        _arm.setSpeed(0.0);
        _stilt.drive(0.0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

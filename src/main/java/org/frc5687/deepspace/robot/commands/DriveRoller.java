package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;

public class DriveRoller extends OutliersCommand {

    private CargoIntake _intake;
    private OI _oi;

    public DriveRoller(Robot robot) {
        _intake = robot.getCargoIntake();
        _oi = robot.getOI();
        requires(_intake);
    }
    @Override
    protected void execute() {
        double speed = _oi.getRollerSpeed();
        _intake.run(speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

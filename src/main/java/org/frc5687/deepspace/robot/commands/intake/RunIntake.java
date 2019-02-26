package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;

public class RunIntake extends OutliersCommand {

    private CargoIntake _intake;
    private OI _oi;

    public RunIntake (Robot robot, CargoIntake intake) {
        _intake = intake;
        _oi = robot.getOI();
        requires(_intake);

    }
    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        double speed = _oi.getRollerSpeed();
        _intake.run(speed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

}

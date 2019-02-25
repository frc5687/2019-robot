package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;

public class CargoIntakeUp extends OutliersCommand {
    private CargoIntake _intake;
    private Robot _robot;
    private long _startTime;

    public CargoIntakeUp(Robot robot) {
        _intake = robot.getCargoIntake();
        _robot = robot;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Intake.RAISE_WRIST_MILLI_SEC;
    }
    @Override
    protected void initialize() {
        _startTime = System.currentTimeMillis();
    }
    @Override
    protected void execute(){
        _intake.raiseWrist();
    }
}

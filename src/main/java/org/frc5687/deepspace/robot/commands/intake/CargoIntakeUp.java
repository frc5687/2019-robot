package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;

public class CargoIntakeUp extends OutliersCommand {
    private CargoIntake _cargoIntake;
    private Robot _robot;
    private long _stopTime;

    public CargoIntakeUp(CargoIntake cargoIntake) {
        _cargoIntake = cargoIntake;
        requires(_cargoIntake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _stopTime;
    }
    @Override
    protected void initialize() {
        _stopTime = System.currentTimeMillis() + Constants.Intake.RAISE_WRIST_MILLI_SEC;
    }
    @Override
    protected void execute(){
        _cargoIntake.raiseWrist();
    }
}

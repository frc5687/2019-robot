package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;

public class CargoIntakeDown extends OutliersCommand {
    private CargoIntake _cargoIntake;
    private long _startTime;

    public CargoIntakeDown(CargoIntake cargoIntake) {
        _cargoIntake = cargoIntake;
        requires(_cargoIntake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Intake.LOWER_WRIST_MILLI_SEC;
    }
    @Override
    protected void initialize() {
        _startTime = System.currentTimeMillis();
        }
    @Override
    protected void execute(){
        _cargoIntake.lowerWrist();
    }
}

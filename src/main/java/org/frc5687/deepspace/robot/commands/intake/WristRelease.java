package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;

public class WristRelease extends OutliersCommand {
    public CargoIntake _intake;

    public WristRelease(CargoIntake intake) {
        _intake = intake;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return true;
    }
    @Override
    protected void initialize() {
    }
    @Override
    protected void execute(){
        _intake.releaseWrist();
    }
}

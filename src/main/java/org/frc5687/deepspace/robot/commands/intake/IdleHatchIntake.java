package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.HatchIntake;

public class IdleHatchIntake extends OutliersCommand {
    public HatchIntake _hatchIntake;

    public IdleHatchIntake(HatchIntake hatchIntake) {
        _hatchIntake = hatchIntake;
        requires(_hatchIntake);
    }
    @Override
    protected void initialize() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute(){
    }
}
package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;
import org.frc5687.deepspace.robot.subsystems.HatchIntake;

public class HoldClawOpen extends OutliersCommand {
    private Robot _robot;
    private HatchIntake _hatchIntake;

    public HoldClawOpen(Robot robot) {
        _robot = robot;
        _hatchIntake = robot.getHatchIntake();
        requires(_hatchIntake);
    }
    @Override
    protected void initialize() {
        if (!_hatchIntake.isHatchDetected() && !_hatchIntake.isShockTriggered()) {
            _hatchIntake.pointClaw();
        }
    }

    @Override
    protected boolean isFinished() {
        return _hatchIntake.isHatchDetected() || _hatchIntake.isShockTriggered();
    }

    @Override
    protected void end() {
            _hatchIntake.gripClaw();
    }

    @Override
    protected void execute(){
    }
}

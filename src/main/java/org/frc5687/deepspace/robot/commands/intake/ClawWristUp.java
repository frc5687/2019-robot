package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.CargoIntake;
import org.frc5687.deepspace.robot.subsystems.HatchIntake;

public class ClawWristUp extends OutliersCommand {
    private HatchIntake _intake;
    private Robot _robot;
    private long _endTime;

    public ClawWristUp(Robot robot) {
        _intake = robot.getHatchIntake();
        _robot = robot;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _endTime;
    }
    @Override
    protected void initialize() {
        _endTime = _intake.isUp() ? System.currentTimeMillis() : System.currentTimeMillis() + Constants.Intake.CLAW_RAISE_WRIST_MILLI_SEC;
        _intake.raiseWrist();
    }
    @Override
    protected void execute(){
        _intake.raiseWrist();
    }
}

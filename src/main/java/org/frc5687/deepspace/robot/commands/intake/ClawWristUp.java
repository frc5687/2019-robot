package org.frc5687.deepspace.robot.commands.intake;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.OutliersCommand;
import org.frc5687.deepspace.robot.subsystems.Intake;

public class ClawWristUp extends OutliersCommand {
    private Intake _intake;
    private Robot _robot;
    private long _startTime;

    public ClawWristUp(Robot robot) {
        _intake = robot.getIntake();
        _robot = robot;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Intake.CLAW_RAISE_WRIST_MILLI_SEC;
    }
    @Override
    protected void initialize() {
        _startTime = System.currentTimeMillis();
    }
    @Override
    protected void execute(){
        _intake.raiseClawWrist();
    }
}

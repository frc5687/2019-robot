package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Intake;
import org.frc5687.deepspace.robot.subsystems.Wrist;

public class WristDown extends OutliersCommand {
    private Intake _intake;
    private Robot _robot;
    private long _startTime;

    public WristDown(Robot robot) {
        _intake = robot.getIntake();
        _robot = robot;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Wrist.LOWER_MILLI_SEC;
    }
    @Override
    protected void initialize() {
        _startTime = System.currentTimeMillis();
        }
    @Override
    protected void execute(){
        _intake.lowerWrist();
    }
}

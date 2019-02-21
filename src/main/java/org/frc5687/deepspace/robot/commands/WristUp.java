package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Intake;
import org.frc5687.deepspace.robot.subsystems.Wrist;

public class WristUp extends OutliersCommand {
    private Intake _intake;
    private Robot _robot;
    private boolean _done = false;
    private long _startTime;

    public WristUp(Robot robot, Intake intake) {
        _intake = intake;
        _robot = robot;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _startTime + Constants.Wrist.RAISE_MILLI_SEC;
    }
    @Override
    protected void initialize() {
        _done = false;
        _startTime = System.currentTimeMillis();
        _robot.setConfiguration(Robot.Configuration.cargo);
        metric("Mode", _robot.getConfiguration().toString());
    }
    @Override
    protected void execute(){
        _intake.raiseWrist();
        _done = true;
    }
}

package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Intake;

import static org.frc5687.deepspace.robot.Constants.Intake.ROLLER_SPEED;
import static org.frc5687.deepspace.robot.subsystems.Intake.RollerMode.DONE;
import static org.frc5687.deepspace.robot.subsystems.Intake.RollerMode.WAITING;

public class StartRoller extends OutliersCommand {
    private Intake _intake;
    private boolean _stopWhenSecured;
    private long _time;

    public StartRoller(Intake intake, boolean stopWhenSecured) {
        _intake = intake;
        _stopWhenSecured = stopWhenSecured;
        requires(_intake);
    }
    
    @Override
    protected void initialize() {
        _intake.setRollerMode(Intake.RollerMode.RUNNING);
        _intake.startRoller();
    }

    @Override
    protected void execute() {
        Intake.RollerMode rollerMode = _intake.getRollerMode();
        switch(rollerMode) {
            case RUNNING:
                _intake.run(ROLLER_SPEED);
                if (_intake.isBallDetected()) {
                    _intake.run(0);
                    _time = System.currentTimeMillis();
                    _intake.setRollerMode(WAITING);
                }
                break;
            case WAITING:
                if (System.currentTimeMillis() > _time + Constants.Roller.TIME_MILLI_SEC) {
                    _intake.setRollerMode(DONE);
                }
        }
    }

    @Override
    protected void end() {
        if (_intake.getRollerMode() == Intake.RollerMode.DONE) {
            _intake.stopRoller();
        }
    }


    @Override
    protected boolean isFinished() {
        if (!_stopWhenSecured) {
            return true;
        }
        return _intake.isBallDetected();
    }
}

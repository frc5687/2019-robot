package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Roller;

import static org.frc5687.deepspace.robot.subsystems.Roller.RollerMode.DONE;
import static org.frc5687.deepspace.robot.subsystems.Roller.RollerMode.WAITING;

public class StartRoller extends OutliersCommand {
    private Roller _roller;
    private boolean _stopWhenSecured = false;
    private long _time;

    public StartRoller(Roller roller, boolean stopWhenSecured) {
        _roller = roller;
        _stopWhenSecured = stopWhenSecured;
        requires(_roller);
    }
    
    @Override
    protected void initialize() {
        _roller.setRollerMode(Roller.RollerMode.RUNNING);
        _roller.start();
    }

    @Override
    protected void execute() {
        Roller.RollerMode rollerMode = _roller.getRollerMode();
        switch(rollerMode) {
            case RUNNING:
                _roller.setSpeed(Constants.Roller.INTAKE_SPEED);
                if (_roller.isBallDetected()) {
                    _roller.setSpeed(0);
                    _time = System.currentTimeMillis();
                    _roller.setRollerMode(WAITING);
                }
                break;
            case WAITING:
                if (System.currentTimeMillis() > _time + Constants.Roller.TIME_MILLI_SEC) {
                    _roller.setRollerMode(DONE);
                }
        }
    }

    @Override
    protected void end() {
        if (_roller.getRollerMode() == Roller.RollerMode.DONE) {
            _roller.stop();
        }
    }


    @Override
    protected boolean isFinished() {
        if (!_stopWhenSecured) {
            return true;
        }
        return _roller.isBallDetected();
    }
}

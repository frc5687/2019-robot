package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Roller;

public class StartRoller extends OutliersCommand {
    private Roller _roller;
    private boolean _stopWhenSecured = false;

    public StartRoller(Roller roller, boolean stopWhenSecured) {
        _roller = roller;
        _stopWhenSecured = stopWhenSecured;
        requires(_roller);
    }
    
    @Override
    protected void initialize() {
        _roller.start();
    }

    @Override
    protected void execute() {
        _roller.setSpeed(Constants.Roller.INTAKE_SPEED);
    }

    @Override
    protected void end() {
        if (_stopWhenSecured) {
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

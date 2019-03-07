package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;

public class Safeguard extends OutliersCommand{
    private Robot _robot;
    private Command _command;
    private long _timeLimit;
    // private CommandGroup _commandGroup;

    public Safeguard(Robot robot, Command command, long timeLimit) {
        _robot = robot;
        _command = command;
        _timeLimit = timeLimit;
    }

//    public Safeguard(Robot robot, CommandGroup commandGroup) {
//        _robot = robot;
//        _commandGroup = commandGroup;
//    }


    @Override
    protected void initialize() {
//        if (DriverStation.getInstance().isFMSAttached()) {
            double timeLeft = DriverStation.getInstance().getMatchTime();
            if (DriverStation.getInstance().isOperatorControl() && _timeLimit < 0) {
                // Only trigger in last _timeLimit seconds...
                if (timeLeft <= -_timeLimit) {
                    _command.start();
                } else {
                    DriverStation.reportError("Skipping command with " + timeLeft + " left when " + _timeLimit + " passed", false);
                }
            } else {
                DriverStation.reportError("Skipping command because not in teleop when " + _timeLimit + " passed", false);
            }
//        }
    }

    @Override
    public void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}


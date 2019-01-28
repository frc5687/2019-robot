package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.deepspace.robot.utils.ILoggingSource;
import org.frc5687.deepspace.robot.utils.RioLogger;

public abstract class OutliersCommand extends Command implements ILoggingSource {
    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }
}

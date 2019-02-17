package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.utils.ILoggingSource;
import org.frc5687.deepspace.robot.utils.RioLogger;

public abstract class OutliersProxy implements ILoggingSource {
    private MetricTracker _metricTracker = new MetricTracker();

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


    public MetricTracker getMetricTracker() {
        return _metricTracker;
    }
    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        _metricTracker.put(name, value);

    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        _metricTracker.put(name, value);

    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
        _metricTracker.put(name, value);

    }

    public abstract void updateDashboard();


}

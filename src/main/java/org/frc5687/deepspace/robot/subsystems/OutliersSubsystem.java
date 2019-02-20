package org.frc5687.deepspace.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.utils.ILoggingSource;
import org.frc5687.deepspace.robot.utils.MetricTracker;
import org.frc5687.deepspace.robot.utils.RioLogger;

public abstract class OutliersSubsystem extends Subsystem implements ILoggingSource {
    private MetricTracker _metricTracker = MetricTracker.createMetricTracker(this);

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

    // Example of metrics collection. In a child class's constructor:
    // getMetricTracker().registerReportableMetricName("foo");
    // getMetricTracker().registerReportableMetricName("bar");
    // getMetricTracker().registerReportableMetricName("baz");
    //
    // Later on in the child class...
    // metric("foo", 123);
    // metric("bar", "elvis");
    // metric("baz", 42.42);
    // metric("pants", 99);    <~ This metric won't get written to USB storage because it wasn't registered.

    protected MetricTracker getMetricTracker() {
        return _metricTracker;
    }
    
    public abstract void updateDashboard();
}

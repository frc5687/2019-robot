package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.robot.commands.Drive;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;

public class MetricTracker {
    private List<String> _reportableMetricNames = new ArrayList<>();
    private Map<String, Object> _rowMetrics = new HashMap<>();
    private static List<MetricTracker> _allMetricsTrackers = new ArrayList<>();
    private String _instrumentedClassName;
    private boolean _streamOpen = false;
    private BufferedWriter _bufferedWriter;

    /**
     * Private ctor. Call createMetricsTracker.
     * @param instrumentedClassName The name of the instrumented class. This is used to name the output file.
     */
    private MetricTracker(String instrumentedClassName, boolean header) {
        // Don't use the c'tor. Use createMetricTracker.
        _instrumentedClassName = instrumentedClassName;
        String outputDir = "/U/"; // USB drive is mounted to /U on roboRIO
        String filename = outputDir + "metrics_" + this._instrumentedClassName + "_" + getDateTimeString() + ".csv";

        try {
            _bufferedWriter = new BufferedWriter(new FileWriter(filename, true));
            _streamOpen = true;
            if (header) {
                _bufferedWriter.write(buildHeaderRow());
            }
        } catch (IOException e) {
            System.out.println("Error initializing metrics file: " + e.getMessage());
        }
    }

    /**
     * MetricsTracker factory method - Creates a new Metrics Tracker and registers the associated object with a list
     * of metrics so they can all be flushed to SD by calling the static flushAll method.
     * @param header if true, writes a header row.
     * @return a fresh new MetricTracker.
     */
    public static MetricTracker createMetricTracker(String instrumentedClassName, boolean header) {
        MetricTracker newMetricTracker = new MetricTracker(instrumentedClassName, header);
        MetricTracker._allMetricsTrackers.add(newMetricTracker);
        return newMetricTracker;
    }

    /**
     * Shorthand version of the ctor
     * @param instrumentedObject
     * @return
     */
    public static MetricTracker createMetricTracker(Object instrumentedObject) {
        return createMetricTracker(instrumentedObject.getClass().getSimpleName(), true);
    }

    /**
     * Adds a metric (name,value) pair to this row of metrics
     * @param name
     * @param value
     */
    public void put(String name, Object value) {
        _rowMetrics.put(name, value);
    }

    /**
     * Only metrics whose name is registered will be written to disk when flushed. Register
     * your metric name in the constructor of whatever class you're instrumenting.
     * @param name
     */
    public void registerReportableMetricName(String name) {
        _reportableMetricNames.add(name);
    }

    /**
     * Starts a new row of metrics. You'd call this, e.g., once per loop.
     */
    public void newMetricRow() {
        if (!_streamOpen) {
            return;
        }
        try {
            DriverStation.getInstance().reportWarning(">>>>> WRITING A NEW METRIC ROW " + buildMetricRow(), false);
            _bufferedWriter.write(buildMetricRow());
            _bufferedWriter.newLine();
            _rowMetrics.clear();
        } catch (IOException e) {
            System.out.println("Error writing metrics file: " + e.getMessage());
        }
    }

    /**
     * Starts a new row for all instrumented objects. You'd call this, e.g., once per loop.
     */
    public static void newMetricRowAll() {
        for(MetricTracker metricTracker : MetricTracker._allMetricsTrackers) {
            metricTracker.newMetricRow();
        }
    }

    /**
     * Writes the metrics for all the instrumented objects to perm storage.
     */
    public static void flushAll() {
        newMetricRowAll();
        for (MetricTracker metricTracker : MetricTracker._allMetricsTrackers) {
            DriverStation.getInstance().reportError(">>>>>>>>>>> METRIC TRACKER IS " + metricTracker._instrumentedClassName, false);
            metricTracker.flushMetricsTracker();
        }
    }

    /**
     * Before shutdown, flushes all the rows of metrics for this instrumented class to perm storage.
     */
    public void flushMetricsTracker() {
        try {
            _bufferedWriter.flush();
            _bufferedWriter.close();
            _streamOpen = false;
        } catch (IOException e) {
            System.out.println("Error closing metrics file: " + e.getMessage());
        }
    }

    // Formats a row of metrics as a comma-delimited quoted string.
    private String buildMetricRow() {
        StringBuilder sb = new StringBuilder();
        int i = 0;
        System.out.println(">>>>>>>>>>>>>>> " + _rowMetrics.toString());
        for (String name : _reportableMetricNames) {
            sb.append("\"");
            if (_rowMetrics.get(name) == null) {
                sb.append("(null)");
            } else {
                sb.append(_rowMetrics.get(name).toString());
            }
            sb.append("\"");
            if (i++ < _reportableMetricNames.size() - 1) {
                sb.append(",");
            }
        }
        return sb.toString();
    }

    // Formats a row of metric names as a string.
    private String buildHeaderRow() {
        StringBuffer sb = new StringBuffer();
        int i = 0;
        for (String name : _reportableMetricNames) {
            sb.append(name);
            if (i < _reportableMetricNames.size() - 1) {
                sb.append(",");
            }
        }
        return sb.toString();
    }

    // Creates a timestamp to include in the log file name.
    private String getDateTimeString() {
        DateFormat df = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
        df.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        return df.format(new Date());
    }
}

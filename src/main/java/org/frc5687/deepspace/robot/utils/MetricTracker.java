package org.frc5687.deepspace.robot.utils;

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
    private boolean _streamOpen = false;
    private BufferedWriter _bufferedWriter;

    /**
     * Private ctor. Call createMetricsTracker.
     * @param instrumentedClassName The name of the instrumented class. This is used to name the output file.
     */
    private MetricTracker(String instrumentedClassName) {
        // Don't use the c'tor. Use createMetricTracker.
        String outputDir = "/U/"; // USB drive is symlinked to /U on roboRIO
        String filename = outputDir + "metrics_" + instrumentedClassName + "_" + getDateTimeString() + ".csv";

        try {
            _bufferedWriter = new BufferedWriter(new FileWriter(filename, true));
            _streamOpen = true;
        } catch (IOException e) {
            System.out.println("Error initializing metrics file: " + e.getMessage());
        }
    }

    /**
     * MetricsTracker factory method - Creates a new Metrics Tracker and registers the associated object with a list
     * of metrics so they can all be flushed to SD by calling the static flushAll method.
     * @param instrumentedClassName name of the class being measured - used to generate the metric file name.
     * @return a fresh new MetricTracker.
     */
    public static MetricTracker createMetricTracker(String instrumentedClassName) {
        MetricTracker newMetricTracker = new MetricTracker(instrumentedClassName);
        MetricTracker._allMetricsTrackers.add(newMetricTracker);
        return newMetricTracker;
    }

    /**
     * Shorthand version of the ctor
     * @param instrumentedObject name of the class being measured - used to generate the metric file name.
     * @return
     */
    public static MetricTracker createMetricTracker(Object instrumentedObject) {
        return createMetricTracker(instrumentedObject.getClass().getSimpleName());
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
     * Starts a new row of metrics. You'd call this, e.g., once per tick.
     */
    public void newMetricRow() {
        if (!_streamOpen || _rowMetrics.isEmpty()) {
            return;
        }
        try {
            _bufferedWriter.write(buildMetricRow());
            _bufferedWriter.newLine();
            _rowMetrics = new HashMap<>();
        } catch (IOException e) {
            System.out.println("Error writing metrics file: " + e.getMessage());
        }
    }

    /**
     * Starts a new row for all instrumented objects. You'd call this, e.g., once per tick.
     */
    public static void newMetricRowAll() {
        for (MetricTracker metricTracker : MetricTracker._allMetricsTrackers) {
            metricTracker.newMetricRow();
        }
    }

    /**
     * Called by a notifier to periodically flush all all known metrics trackers the rows of metrics
     * to perm storage.
     */
    public static void flushAll() {
        for (MetricTracker metricTracker : MetricTracker._allMetricsTrackers) {
            metricTracker.flushMetricsTracker();
        }
    }

    /**
     * Flushes the buffer of stats for an instance of a metrics tracker to perm storage.
     */
    public void flushMetricsTracker() {
        try {
            _bufferedWriter.flush();
        } catch (IOException e) {
            System.out.println("Error closing metrics file: " + e.getMessage());
        }
    }

    // Formats a row of metrics as a comma-delimited quoted string.
    private String buildMetricRow() {
        StringBuilder sb = new StringBuilder();
        for (String name : _reportableMetricNames) {
            sb.append("\"");
            if (_rowMetrics.get(name) == null) {
                sb.append("(null)");
            } else {
                sb.append(_rowMetrics.get(name).toString());
            }
            sb.append("\"");
            int i = 0;
            if (i++ < _reportableMetricNames.size() - 1) {
                sb.append(",");
            }
        }
        return sb.toString();
    }

    // Creates a timestamp to include in the log file name.
    private String getDateTimeString() {
        DateFormat df = new SimpleDateFormat("yyyyMMdd_HHmmss");
        df.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        return df.format(new Date());
    }
}
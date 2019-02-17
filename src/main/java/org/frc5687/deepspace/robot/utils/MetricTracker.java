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
    private List<Map<String, Object>> _allMetrics = new ArrayList<>();

    /**
     * Adds a metric name,value pair to this row of metrics
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
        _allMetrics.add(_rowMetrics);
        _rowMetrics.clear();
    }

    /**
     * Before shutdown, flushes all the rows of metrics to disk.
     * @param header if true, write a header row with the metric names
     * @return 0 on success, -1 if an exception go thrown while flushing.
     */
    public int flushReportableMetrics(boolean header) {
        String outputDir = "/U/"; // USB drive is mounted to /U on roboRIO
        String filename = outputDir + "metrics_" + this.getClass().getSimpleName() + getDateTimeString() + ".csv";

        try {
            FileWriter fileWriter = new FileWriter(filename, true);
            BufferedWriter logFile = new BufferedWriter(fileWriter);
            if (header) {
                logFile.write(buildHeaderRow());
            }
            for (Map<String, Object> row : _allMetrics) {
                logFile.write(buildMetricRow(row));
            }
            fileWriter.close();
        } catch (IOException e) {
            System.out.println("Error initializing log file: " + e.getMessage());
            return -1;
        }
        return 0;
    }

    // Formats a row of metrics as a comma-delimited quoted string.
    private String buildMetricRow(Map<String, Object> row) {
        StringBuffer sb = new StringBuffer();
        int i = 0;
        for (String name : _reportableMetricNames) {
            sb.append("\"");
            sb.append(row.get(name).toString());
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

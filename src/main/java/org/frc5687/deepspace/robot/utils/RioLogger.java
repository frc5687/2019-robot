package org.frc5687.deepspace.robot.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

//Imported from RobotCasserole2017
public class RioLogger {
    private static RioLogger _instance;

    public static RioLogger getInstance() {
        if (_instance==null) {
            _instance = new RioLogger();
        }
        return _instance;
    }

    public static void warn(String source, String message) {
        log("warn", source, message);
    }
    public static void error(String source, String message) {
        log("error", source, message);
    }
    public static void debug(String source, String message) {
        log("debug", source, message);
    }
    public static void info(String source, String message) {
        log("info", source, message);
    }

    public static void warn(Object source, String message) { warn(source.getClass().getSimpleName(), message); }
    public static void error(Object source, String message) { error(source.getClass().getSimpleName(), message); }
    public static void debug(Object source, String message) { debug(source.getClass().getSimpleName(), message); }
    public static void info(Object source, String message) { info(source.getClass().getSimpleName(), message); }

    public static void log(String level, String source, String message) {
        getInstance().writeData( getInstance().getDateTimeString(), level, source, message);
    }

    private FileWriter fwriter;
    long log_write_index;
    String log_name = null;
    String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
    BufferedWriter log_file = null;
    boolean log_open = false;

    public int init() {

        if (log_open) {
            System.out.println("Warning - log is already open!");
            return 0;
        }

        log_open = false;
        try {
            // Reset state variables
            log_write_index = 0;

            // Determine a unique file name
            log_name = output_dir + "log_" + getDateTimeString() + ".txt";
            File fileText = new File(log_name);
            // Open File
            FileWriter fstream = new FileWriter(log_name, true);
            log_file = new BufferedWriter(fstream);
            // End of line
            log_file.write("\n");
            log_open = true;

        }
        // Catch ALL the errors!!!
        catch (IOException e) {

            System.out.println("Error initializing log file: " + e.getMessage());
            return -1;
        }
        
        log_open = true;

        return 0;
    }

    private String getDateTimeString() {
        DateFormat df = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
        df.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        return df.format(new Date());
    }

    public int writeData(String... data_elements) {
        String line_to_write = "";

        if (log_open == false) {
            System.out.println("Error - Log is not yet opened, cannot write!");
            return -1;
        }

        try {

            // Write user-defined data
            for (String data_val : data_elements) {
                line_to_write += data_val + " ";
            }

            // End of line
            line_to_write = line_to_write + "\n";

            // write constructed string out to file

            log_file.write(line_to_write);


        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error writing to log file: " + e.getMessage());
            return -1;
        }

        log_write_index++;
        return 0;
    }



    /**
     * Clears the buffer in memory and forces things to file. Generally a good idea to use this as
     * infrequently as possible (because it increases logging overhead), but definitely use it
     * before the roboRIO might crash without a proper call to the close() method (during brownout,
     * maybe?)
     *
     * @return Returns 0 on flush success or -1 on failure.
     */
    public int forceSync() {
        if (log_open == false) {
            System.out.println("Error - Log is not yet opened, cannot sync!");
            return -1;
        }
        try {
            log_file.flush();
        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error flushing IO stream file: " + e.getMessage());
            return -1;
        }

        return 0;

    }



    /**
     * Closes the log file and ensures everything is written to disk. init() must be called again in
     * order to write to a new file.
     *
     * @return -1 on failure to close, 0 on success
     */
    public int close() {

        if (log_open == false) {
            System.out.println("Warning - Log is not yet opened, nothing to close.");
            return 0;
        }

        try {
            log_file.close();
            log_open = false;
        }
        // Catch ALL the errors!!!
        catch (IOException e) {
            System.out.println("Error Closing Log File: " + e.getMessage());
            return -1;
        }
        return 0;
    }
}


package org.firstinspires.ftc.teamcode.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * A very simple logger intended to mimic the behaviour of the PsiKit library.
 *
 * <p>This adapter writes a CSV file to persistent storage on the robot.  Each
 * record contains the system time in milliseconds, a key and a value.  The
 * format is intentionally simple so that it can be imported into
 * AdvantageScope as a CSV log.  By matching the key names used in
 * {@link TelemetryPublisher} you can visualise the same values live in the
 * dashboard and offline in AdvantageScope.  If you integrate the actual
 * PsiKit library later, you can swap this implementation without changing
 * calling code.</p>
 */
public class PsiKitAdapter {

    private FileWriter writer;
    private boolean active;

    /**
     * Starts a new logging session.  Creates a directory on the SD card and
     * opens a CSV file named with the current timestamp.  If called again
     * while a session is active the existing session will be closed first.
     */
    public void startSession() {
        stopSession();
        try {
            // Create a directory under FIRST for logs
            File logDir = new File("/sdcard/FIRST/PsiKitLogs");
            if (!logDir.exists() && !logDir.mkdirs()) {
                // If we cannot create the directory, disable logging
                return;
            }
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
            File logFile = new File(logDir, "log_" + timestamp + ".csv");
            writer = new FileWriter(logFile);
            // CSV header
            writer.write("time_ms,key,value\n");
            writer.flush();
            active = true;
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Records a numeric value.  Values are written with millisecond timestamps.
     *
     * @param key the name of the metric
     * @param value the value to record
     */
    public void recordNumber(String key, double value) {
        if (!active) return;
        try {
            long time = System.currentTimeMillis();
            writer.write(time + "," + key + "," + value + "\n");
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Records a boolean value.  Booleans are encoded as 1 or 0.
     *
     * @param key the name of the metric
     * @param value the value to record
     */
    public void recordBoolean(String key, boolean value) {
        recordNumber(key, value ? 1.0 : 0.0);
    }

    /**
     * Records a string value.  Strings are quoted to preserve commas.
     *
     * @param key the name of the metric
     * @param value the string to record
     */
    public void recordString(String key, String value) {
        if (!active) return;
        try {
            long time = System.currentTimeMillis();
            // Wrap string in quotes and escape any quotes inside
            String safe = value.replace("\"", "\"\"");
            writer.write(time + "," + key + ",\"" + safe + "\"\n");
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Flushes the underlying writer to ensure that all buffered data is
     * persisted to disk.  Flushing frequently can reduce the risk of data
     * loss on crash at the expense of performance.
     */
    public void flush() {
        try {
            if (writer != null) {
                writer.flush();
            }
        } catch (IOException e) {
            e.printStackTrace();
            active = false;
        }
    }

    /**
     * Stops the current session and closes the file.  Subsequent records will
     * be ignored until a new session is started.
     */
    public void stopSession() {
        try {
            if (writer != null) {
                writer.flush();
                writer.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            writer = null;
            active = false;
        }
    }
}
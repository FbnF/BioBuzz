package org.firstinspires.ftc.teamcode.utils;

/**
 * Simple time helpers for keeping the main loop clean.
 */
public class TimeUtils {

    private long startTimeNs = 0;
    private boolean initialized = false;

    /**
     * Resets the timer. Call this right before you want to start counting.
     */
    public void reset() {
        startTimeNs = System.nanoTime();
        initialized = true;
    }

    /**
     * Checks if a certain amount of seconds has passed since reset().
     */
    public boolean secondsPassed(double seconds) {
        if (!initialized) return false;
        double elapsed = (System.nanoTime() - startTimeNs) / 1e9;
        return elapsed >= seconds;
    }

    /**
     * Converts milliseconds to nanoseconds.
     */
    public static long msToNs(long ms) {
        return ms * 1_000_000L;
    }
}

package org.firstinspires.ftc.teamcode.utils;

/**
 * PDController - A simple Proportional-Derivative controller.
 * We use this for tasks like Auto-Aligning to a vision tag.
 */
public class PDController {

    private double kp;
    private double kd;
    private double lastError = 0.0;
    private long lastTimeNs = 0L;

    public PDController(double kp, double kd) {
        this.kp = kp;
        this.kd = kd;
    }

    /**
     * Calculates the required correction power based on current error.
     */
    public double update(double error) {
        long now = System.nanoTime();
        
        // Calculate the time difference (dt)
        double dt = (lastTimeNs == 0L) ? 0.0 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        // Calculate the rate of change (derivative)
        double derivative = 0.0;
        if (dt > 1e-4) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        // PD Formula: (P * error) + (D * derivative)
        return (kp * error) + (kd * derivative);
    }

    /**
     * Resets the controller's memory.
     */
    public void reset() {
        lastError = 0.0;
        lastTimeNs = 0L;
    }

    public void setGains(double kp, double kd) {
        this.kp = kp;
        this.kd = kd;
    }
}

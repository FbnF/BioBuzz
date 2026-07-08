package org.firstinspires.ftc.teamcode.NewTeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * LedService - The robot's "Voice."
 * It communicates with the driver using the Puck Light colors.
 */
public class LedService {

    private static LedService instance = null;
    
    // Status positions (colors)
    private static final double SEARCHING = 0.287; // Usually Purple or Blue
    private static final double BLOCKED = 0.368;   // Yellow (No-shot zone or spinning up)
    private static final double READY = 0.622;     // Green (Ready to fire!)
    private static final double OFF = 0.0;

    // Flash/Warning variables
    private ElapsedTime flashTimer = new ElapsedTime();
    private boolean isWarning = false;
    private int flashCount = 0;
    private boolean flashState = false;

    public static LedService getInstance() {
        if (instance == null) {
            instance = new LedService();
        }
        return instance;
    }

    /**
     * Updates the LED color based on the robot's current state.
     */
    public void setStatus(boolean targetVisible, boolean atSpeed, boolean noShotZone) {
        // If we're in the middle of a "Too Soon" flash warning, don't interrupt it
        if (isWarning) return;

        RobotHardware robot = RobotHardware.getInstance();
        if (robot.puckLight == null) return;

        if (!targetVisible) {
            robot.puckLight.setPosition(SEARCHING);
        } else if (noShotZone || !atSpeed) {
            robot.puckLight.setPosition(BLOCKED);
        } else if (atSpeed) {
            robot.puckLight.setPosition(READY);
        }
    }

    /**
     * Triggers the "Denied" flashing yellow warning.
     */
    public void triggerWarning() {
        if (!isWarning) {
            isWarning = true;
            flashCount = 0;
            flashTimer.reset();
        }
    }

    /**
     * Handles the flashing logic. Call this every loop.
     */
    public void update() {
        if (!isWarning) return;

        RobotHardware robot = RobotHardware.getInstance();
        if (flashTimer.milliseconds() > 200) {
            flashState = !flashState;
            robot.puckLight.setPosition(flashState ? BLOCKED : OFF);
            flashTimer.reset();
            
            if (!flashState) flashCount++;
            if (flashCount >= 5) isWarning = false; // Stop after 5 blinks
        }
    }
}

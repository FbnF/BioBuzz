package org.firstinspires.ftc.teamcode.NewTeleOp.Services;

import com.qualcomm.robotcore.util.ElapsedTime;

// Status light logic
public class LedService {

    private static LedService instance = null;
    
    // LED positions
    private static final double SEARCHING = 0.287; 
    private static final double BLOCKED = 0.368;   
    private static final double READY = 0.622;     
    private static final double OFF = 0.0;

    private ElapsedTime flashTimer = new ElapsedTime();
    private boolean isWarning = false;
    private int flashCount = 0;
    private boolean flashState = false;

    public static LedService getInstance() {
        if (instance == null) instance = new LedService();
        return instance;
    }

    public void setStatus(boolean targetVisible, boolean atSpeed, boolean noShotZone) {
        if (isWarning) return;
        RobotHardware robot = RobotHardware.getInstance();
        if (robot.puckLight == null) return;

        if (!targetVisible) {
            robot.puckLight.setPosition(SEARCHING);
        } else if (noShotZone || !atSpeed) {
            robot.puckLight.setPosition(BLOCKED);
        } else {
            robot.puckLight.setPosition(READY);
        }
    }

    public void triggerWarning() {
        if (!isWarning) {
            isWarning = true;
            flashCount = 0;
            flashTimer.reset();
        }
    }

    public void update() {
        if (!isWarning) return;
        RobotHardware robot = RobotHardware.getInstance();
        if (flashTimer.milliseconds() > 200) {
            flashState = !flashState;
            robot.puckLight.setPosition(flashState ? BLOCKED : OFF);
            flashTimer.reset();
            if (!flashState) flashCount++;
            if (flashCount >= 5) isWarning = false;
        }
    }
}

package org.firstinspires.ftc.teamcode.NewTeleOp.Services;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

import java.util.List;

// Vision logic
public class VisionService {

    private Limelight3A limelight;
    private double filteredDistance = 0.0;
    private double rawDistance = 0.0;
    private boolean targetVisible = false;
    private boolean hasGoalTag = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private int goalTagId = 24;
    private boolean visionEnabled = true;
    
    private long lastSeenNs = 0;
    private static final long NULL_DELAY_NS = 1_000_000_000L;

    private static boolean currentlySeeing;

    private static VisionService instance = null;

    public static VisionService getInstance() {
        if (instance == null) instance = new VisionService();
        return instance;
    }

    public void setGoalTagId(int id) { this.goalTagId = id; }
    public void setVisionEnabled(boolean enabled) { this.visionEnabled = enabled; }
    public boolean isVisionEnabled() { return visionEnabled; }

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void update() {
        if (!visionEnabled) {
            targetVisible = false;
            hasGoalTag = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        currentlySeeing = false;
        hasGoalTag = false;

        if (result != null && result.isValid() && result.getStaleness() < 100) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f != null && f.getFiducialId() == goalTagId) {
                        Pose3D pose = f.getCameraPoseTargetSpace();
                        if (pose != null) {
                            rawDistance = Math.sqrt(Math.pow(pose.getPosition().x, 2) + Math.pow(pose.getPosition().z, 2)) * 39.3701;
                            
                            // Smoothing
                            if (filteredDistance <= 0.0) filteredDistance = rawDistance;
                            double a = ShooterConfig.DIST_SMOOTH_ALPHA;
                            filteredDistance = (1.0 - a) * filteredDistance + a * rawDistance;

                            tx = f.getTargetXDegrees();
                            ty = f.getTargetYDegrees();
                            currentlySeeing = true;
                            hasGoalTag = true;
                            lastSeenNs = System.nanoTime();
                            break;
                        }
                    }
                }
            }
        }

        // Null delay buffer
        if (currentlySeeing) {
            targetVisible = true;
        } else {
            targetVisible = (System.nanoTime() - lastSeenNs) < NULL_DELAY_NS;
        }
    }

    public boolean isNoShotZone() {
        return targetVisible && ShooterConfig.NO_SHOT_UNDER_IN > 0.0 && filteredDistance < ShooterConfig.NO_SHOT_UNDER_IN;
    }

    public double[] getTxWindow() {
        if (goalTagId == 20) { // Blue
            return interpolateWindow(filteredDistance, ShooterConfig.DIST_IN_BLUE, ShooterConfig.MIN_ANGLE_BLUE, ShooterConfig.MAX_ANGLE_BLUE);
        } else { // Red
            return interpolateWindow(filteredDistance, ShooterConfig.DIST_IN_RED, ShooterConfig.MIN_ANGLE_RED, ShooterConfig.MAX_ANGLE_RED);
        }
    }

    private double[] interpolateWindow(double x, double[] dists, double[] mins, double[] maxs) {
        if (dists == null || mins == null || maxs == null || dists.length < 2) return new double[]{-5, 5};
        
        int i = 0;
        int last = dists.length - 1;
        if (x <= dists[0]) i = 0;
        else if (x >= dists[last]) i = last - 1;
        else {
            while (i < last - 1 && x > dists[i + 1]) i++;
        }

        double x0 = dists[i];
        double x1 = dists[i+1];
        double t = (x - x0) / (x1 - x0);

        return new double[]{
            mins[i] + t * (mins[i+1] - mins[i]),
            maxs[i] + t * (maxs[i+1] - maxs[i])
        };
    }

    public double getDistance() { return filteredDistance; }
    public double getRawDistance() { return rawDistance; }
    public boolean isTargetVisible() { return targetVisible; }
    public boolean hasGoalTag() { return hasGoalTag; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public boolean isCurrentlySeeing() { return currentlySeeing; }
    public int getGoalTagId() { return goalTagId; }
}
